#include "stdafx.h"
#include "Connection.h"
#include <stdio.h>
#if defined(_MSC_VER)
#include <Windows.h>
#include <process.h>
#define LockMutex EnterCriticalSection
#define UnlockMutex LeaveCriticalSection
#else
#include <unistd.h>
#include <pthread.h>
#define LockMutex pthread_mutex_lock
#define UnlockMutex pthread_mutex_unlock
#endif

#if defined(_MSC_VER)
static void serviceMessageLoop(void* unused);
#else
static void* serviceMessageLoop(void* unused);
#endif

#if defined(_MSC_VER)
static HANDLE pollingThread;
static CRITICAL_SECTION dataLock;
#else
static pthread_t pollingThread;
static pthread_mutex_t dataLock;
#endif

static void setFrame(const LEAP_TRACKING_EVENT* frame);
static void setDevice(const LEAP_DEVICE_INFO* deviceProps);

LEAP_CONNECTION ConnectionHandle = NULL;
bool IsConnected = false;
uint32_t DeviceId = 0;

static volatile bool _isRunning = false;
static LEAP_CONNECTION_CONFIG* lastConnectionConfig;
static LEAP_TRACKING_EVENT* lastFrame = NULL;
static LEAP_DEVICE_INFO* lastDevice = NULL;

struct LeapMotionConnection::Callbacks ConnectionCallbacks;

LeapMotionConnection::LeapMotionConnection()
{
}

static bool connectionConfigEq(const LEAP_CONNECTION_CONFIG* connectionConfig1, const LEAP_CONNECTION_CONFIG* connectionConfig2) {
    return connectionConfig1 == NULL ? connectionConfig2 == NULL : connectionConfig2 != NULL
        && connectionConfig1->size == connectionConfig2->size
        && !memcmp(connectionConfig1, connectionConfig2, connectionConfig1->size);
}

LEAP_CONNECTION* LeapMotionConnection::OpenConnectionWithConfig(const LEAP_CONNECTION_CONFIG* connectionConfig) {
    if (connectionConfigEq(connectionConfig, lastConnectionConfig)) {
        if (_isRunning) {
            return &ConnectionHandle;
        }
    }
    else {
        DestroyConnection();
        LEAP_CONNECTION_CONFIG* newConnectionConfig = NULL;
        if (connectionConfig) {
            newConnectionConfig = (LEAP_CONNECTION_CONFIG*) realloc(lastConnectionConfig, connectionConfig->size);
        }
        if (newConnectionConfig == NULL) {
            free(lastConnectionConfig);
        }
        else {
            memcpy(newConnectionConfig, connectionConfig, connectionConfig->size);
        }
        lastConnectionConfig = newConnectionConfig;
    }
    if (ConnectionHandle || LeapCreateConnection(connectionConfig, &ConnectionHandle) == eLeapRS_Success) {
        eLeapRS result = LeapOpenConnection(ConnectionHandle);
        if (result == eLeapRS_Success) {
            _isRunning = true;
#if defined(_MSC_VER)
            InitializeCriticalSection(&dataLock);
            pollingThread = (HANDLE)_beginthread(serviceMessageLoop, 0, NULL);
#else
            pthread_create(&pollingThread, NULL, serviceMessageLoop, NULL);
#endif
        }
    }
    return &ConnectionHandle;
}

void LeapMotionConnection::CloseConnection() {
    if (!_isRunning) {
        return;
    }
    _isRunning = false;
    LeapCloseConnection(ConnectionHandle);
#if defined(_MSC_VER)
    WaitForSingleObject(pollingThread, INFINITE);
    // handle automatically closed when polling thread exits?
    //CloseHandle(pollingThread);
#else
    pthread_join(pollingThread, NULL);
#endif
    IsConnected = false;
}

void LeapMotionConnection::DestroyConnection() {
    CloseConnection();
    LeapDestroyConnection(ConnectionHandle);
    ConnectionHandle = NULL;
}

static void handleConnectionEvent(const LEAP_CONNECTION_EVENT* connection_event) {
    IsConnected = true;
    if (ConnectionCallbacks.on_connection) {
        ConnectionCallbacks.on_connection();
    }
}

void Test()
{
    //LeapPollConnection();

    uint32_t data = 0;
    uint32_t* pnArray = &data;
    LeapGetDeviceList(ConnectionHandle, NULL, pnArray);
    printf("Num of Devices in the DeviceList: %ld\n", *pnArray);
    if (data >= 2)
    {
        LEAP_DEVICE_REF* pArray = (LEAP_DEVICE_REF*) malloc(sizeof(LEAP_DEVICE_REF) * data);
        LeapGetDeviceList(ConnectionHandle, pArray, pnArray);
        for (int i = 0; i < data; i++)
        {
            LEAP_DEVICE device;
            LEAP_DEVICE* deviceHandle = &device;
            LeapOpenDevice(pArray[i], deviceHandle);
            LEAP_DEVICE_INFO deviceInfo;
            deviceInfo.serial_length = 100;
            deviceInfo.serial = (char *) malloc(deviceInfo.serial_length);
            LEAP_DEVICE_INFO* deviceInfoHandle = &deviceInfo;
            LeapGetDeviceInfo(device, deviceInfoHandle);
            printf("Device %d: %ld, serial=%s\n", i, pArray[i].id, deviceInfo.serial);
        }
    }
    return;
}

static void handleDeviceEvent(const LEAP_DEVICE_EVENT* device_event) {
    LEAP_DEVICE deviceHandle;
    //Open device using LEAP_DEVICE_REF from event struct.
    eLeapRS result = LeapOpenDevice(device_event->device, &deviceHandle);
    if (result != eLeapRS_Success) {
        printf("Could not open device %s.\n", LeapMotionConnection::ResultString(result));
        return;
    }
    //Create a struct to hold the device properties, we have to provide a buffer for the serial string
    LEAP_DEVICE_INFO deviceProperties = { sizeof(deviceProperties) };
    // Start with a length of 1 (pretending we don't know a priori what the length is).
    // Currently device serial numbers are all the same length, but that could change in the future
    deviceProperties.serial_length = 1;
    deviceProperties.serial = (char*) malloc(deviceProperties.serial_length);
    //This will fail since the serial buffer is only 1 character long
    // But deviceProperties is updated to contain the required buffer length
    result = LeapGetDeviceInfo(deviceHandle, &deviceProperties);
    if (result == eLeapRS_InsufficientBuffer) {
        //try again with correct buffer size
        deviceProperties.serial = (char *) realloc(deviceProperties.serial, deviceProperties.serial_length);
        result = LeapGetDeviceInfo(deviceHandle, &deviceProperties);
        if (result != eLeapRS_Success) {
            printf("Failed to get device info %s.\n", LeapMotionConnection::ResultString(result));
            free(deviceProperties.serial);
            return;
        }
    }
    setDevice(&deviceProperties);
    if (ConnectionCallbacks.on_device_found) {
        ConnectionCallbacks.on_device_found(&deviceProperties, deviceHandle);
    }

    free(deviceProperties.serial);
    LeapCloseDevice(deviceHandle);
}

static void handleTrackingEvent(const LEAP_TRACKING_EVENT* tracking_event) {
    setFrame(tracking_event); //support polling tracking data from different thread

    if (ConnectionCallbacks.on_frame) {
        ConnectionCallbacks.on_frame(tracking_event);
    }
}

#if defined(_MSC_VER)
static void serviceMessageLoop(void* unused) {
#else
static void* serviceMessageLoop(void* unused) {
#endif
    eLeapRS result;
    LEAP_CONNECTION_MESSAGE msg;
    while (_isRunning) {
        unsigned int timeout = 1000;
        result = LeapPollConnection(ConnectionHandle, timeout, &msg);

        if (result != eLeapRS_Success) {
            printf("LeapC PollConnection call was %s.\n", LeapMotionConnection::ResultString(result));
            continue;
        }

        DeviceId = msg.device_id;

        switch (msg.type) {

        case eLeapEventType_Connection:
            handleConnectionEvent(msg.connection_event);
            break;
        case eLeapEventType_ConnectionLost:
            //handleConnectionLostEvent(msg.connection_lost_event);
            break;
        case eLeapEventType_Device:
            handleDeviceEvent(msg.device_event);
            break;
        case eLeapEventType_DeviceLost:
            //handleDeviceLostEvent(msg.device_event);
            break;
        case eLeapEventType_DeviceFailure:
            //handleDeviceFailureEvent(msg.device_failure_event);
            break;
        case eLeapEventType_Tracking:
            handleTrackingEvent(msg.tracking_event);
            break;
        case eLeapEventType_ImageComplete:
            // Ignore
            break;
        case eLeapEventType_ImageRequestError:
            // Ignore
            break;
        case eLeapEventType_LogEvent:
            //handleLogEvent(msg.log_event);
            break;
        case eLeapEventType_Policy:
            //handlePolicyEvent(msg.policy_event);
            break;
        case eLeapEventType_ConfigChange:
            //handleConfigChangeEvent(msg.config_change_event);
            break;
        case eLeapEventType_ConfigResponse:
            //handleConfigResponseEvent(msg.config_response_event);
            break;
        case eLeapEventType_DeviceStatusChange:
            //handleDeviceStatusChangeEvent(msg.device_status_change_event);
            break;
        case eLeapEventType_DroppedFrame:
            //handleDroppedFrameEvent(msg.dropped_frame_event);
            break;
        case eLeapEventType_Image:
            //handleImageEvent(msg.image_event);
            break;
        case eLeapEventType_PointMappingChange:
            //handlePointMappingChangeEvent(msg.point_mapping_change_event);
            break;
        case eLeapEventType_LogEvents:
            //handleLogEvents(msg.log_events);
            break;
        case eLeapEventType_HeadPose:
            //handleHeadPoseEvent(msg.head_pose_event);
            break;
        default:
            //discard unknown message types
            printf("Unhandled message type %i.\n", msg.type);
        } //switch on msg.type
    }
#if !defined(_MSC_VER)
    return NULL;
#endif
}

void setFrame(const LEAP_TRACKING_EVENT* frame) {
    LockMutex(&dataLock);
    if (!lastFrame) lastFrame = (LEAP_TRACKING_EVENT*) malloc(sizeof(*frame));
    *lastFrame = *frame;
    UnlockMutex(&dataLock);
}

static void setDevice(const LEAP_DEVICE_INFO* deviceProps) {
    LockMutex(&dataLock);
    if (lastDevice) {
        free(lastDevice->serial);
    }
    else {
        lastDevice = (LEAP_DEVICE_INFO*) malloc(sizeof(*deviceProps));
    }
    *lastDevice = *deviceProps;
    lastDevice->serial = (char *) malloc(deviceProps->serial_length);
    memcpy(lastDevice->serial, deviceProps->serial, deviceProps->serial_length);
    UnlockMutex(&dataLock);
}

const char* LeapMotionConnection::ResultString(eLeapRS r) {
    switch (r) {
    case eLeapRS_Success:                  return "eLeapRS_Success";
    case eLeapRS_UnknownError:             return "eLeapRS_UnknownError";
    case eLeapRS_InvalidArgument:          return "eLeapRS_InvalidArgument";
    case eLeapRS_InsufficientResources:    return "eLeapRS_InsufficientResources";
    case eLeapRS_InsufficientBuffer:       return "eLeapRS_InsufficientBuffer";
    case eLeapRS_Timeout:                  return "eLeapRS_Timeout";
    case eLeapRS_NotConnected:             return "eLeapRS_NotConnected";
    case eLeapRS_HandshakeIncomplete:      return "eLeapRS_HandshakeIncomplete";
    case eLeapRS_BufferSizeOverflow:       return "eLeapRS_BufferSizeOverflow";
    case eLeapRS_ProtocolError:            return "eLeapRS_ProtocolError";
    case eLeapRS_InvalidClientID:          return "eLeapRS_InvalidClientID";
    case eLeapRS_UnexpectedClosed:         return "eLeapRS_UnexpectedClosed";
    case eLeapRS_UnknownImageFrameRequest: return "eLeapRS_UnknownImageFrameRequest";
    case eLeapRS_UnknownTrackingFrameID:   return "eLeapRS_UnknownTrackingFrameID";
    case eLeapRS_RoutineIsNotSeer:         return "eLeapRS_RoutineIsNotSeer";
    case eLeapRS_TimestampTooEarly:        return "eLeapRS_TimestampTooEarly";
    case eLeapRS_ConcurrentPoll:           return "eLeapRS_ConcurrentPoll";
    case eLeapRS_NotAvailable:             return "eLeapRS_NotAvailable";
    case eLeapRS_NotStreaming:             return "eLeapRS_NotStreaming";
    case eLeapRS_CannotOpenDevice:         return "eLeapRS_CannotOpenDevice";
    default:                               return "unknown result type.";
    }
}
/** Cross-platform sleep function */
void millisleep(int milliseconds) {
#ifdef _WIN32
    Sleep(milliseconds);
#else
    usleep(milliseconds * 1000);
#endif
}
//End-of-ExampleConnection.c