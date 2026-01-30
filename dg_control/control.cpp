#include "control.hpp"

using namespace handcontrol;

DGControl *DGControl::_instancePtr = nullptr; 

DGControl::DGControl(const char* ip, int port, int slaveID):
_target_joint_buffer(1024),
_current_joint_buffer(1024)
{
    this->_port = port;
    this->_slaveID = slaveID;
    std::memcpy(this->_ip, ip, MAX_GRIPPER_IP_ADDRESS_SIZE);

    _msgTargetPos << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
    _msgCurrentPos << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
}

DGControl* DGControl::getInstance(const char* ip, int port, int slaveID)
{
    if (_instancePtr == nullptr) {
            _instancePtr = new DGControl(ip, port, slaveID);
    }
    return _instancePtr;
}

// --------------------------------------------------------------------------------

void DGControl::_ConnectedCallback()
{
    getInstance()->_g_connected.store(true);
}

void DGControl::_DisconnectedCallback()
{
    getInstance()->_g_connected.store(false);
}

void DGControl::_ReceivedGripperDataCallback(ReceivedGripperData data)
{
    getInstance()->_g_gripperData = data;
}

void DGControl::_CommunicationPeriodCallback(int period)
{
    getInstance()->_g_commPeriod = period;
}

void DGControl::_DiagnosisCallback(DiagnosisSystem diag)
{
    getInstance()->_g_diagnosisData = diag;
}

void DGControl::_FingertipCallback(ReceivedFingertipSensorData data)
{
    getInstance()->_g_sensorData = data;
}

void DGControl::_GPIOCallback(ReceivedGPIOData data)
{
    getInstance()->_g_gpioData = data;
}

void DGControl::_DataProcessingCallback(int status)
{
    getInstance()->_g_processing = status;
}


void DGControl::_setCallbacks()
{
    CallbackForOnConnected(DGControl::_ConnectedCallback);
    CallbackForOnDisconnected(DGControl::_DisconnectedCallback);
    CallbackForOnReceivedGripperData(DGControl::_ReceivedGripperDataCallback);
    CallbackForOnCommunicationPeriod(DGControl::_CommunicationPeriodCallback);
    CallbackForOnDiagnosisSystem(DGControl::_DiagnosisCallback);
    CallbackForOnReceivedFingertipSensorData(DGControl::_FingertipCallback);
    CallbackForOnReceivedGPIOData(DGControl::_GPIOCallback);
    CallbackForOnDataProcessing(DGControl::_DataProcessingCallback);
}

// --------------------------------------------------------------------------------

void DGControl::start()
{
    int success = 0;
    DG_RESULT result;

    GripperSystemSetting setting{};

    setting.communicationMode = COMMUNICATION_MODE_ETHERNET;
    setting.controlMode       = CONTROL_MODE_DEVELOPER;
    setting.port              = this->_port;
    setting.slaveID           = this->_slaveID;
    setting.readTimeout       = this->_readTimeout;
    std::memcpy(setting.ip, this->_ip, MAX_GRIPPER_IP_ADDRESS_SIZE);

    result = SetGripperSystem(setting);
    std::cout << "SetGripperSystem: " << result << "\n";
    success += result;

    // --------------------------

    this->_setCallbacks();

    // --------------------------

    result = ConnectToGripper();
    std::cout << "ConnectToGripper: " << result << "\n";
    success += result;

    while (!_g_connected.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // --------------------------

    GripperSetting gs{};
    gs.model = DG_MODEL_DG_5F_RIGHT;
    gs.movingInpose = 1;
    int type[MAX_RECEIVED_DATA_TYPE_COUNT] = {1,2,3,4,5,6};
    std::memcpy(gs.receivedDataType, type, sizeof(type));

    result = SetGripperOption(gs);
    success += result;
    std::cout << "SetGripperOption: " << result << "\n";

    // --------------------------

    while (_g_commPeriod.load() < 200) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    result = SystemStart();
    std::cout << "SystemStart: " << result << "\n";
    success += result;

    // --------------------------

	for(int i=0;i<MAX_JOINT_COUNT;i++)
	{
		_P[i] = 2.0f;   // типичные безопасные значения
		_D[i] = 2.0f;
	}

    SetJointGainPAll(_P);
	SetJointGainDAll(_D);

    SetMotionTimeAllEqual(300);

    // --------------------------

    std::memcpy(_tempPos, _g_gripperData.joint, sizeof(_g_gripperData.joint));

    _control = std::jthread(&DGControl::_loop, this);
}

void DGControl::stop()
{
    if (_g_connected.load()) 
    {
        MoveJointAll(_currentPos); // Чтобы пальцы после выключения не двигались

        _g_connected.store(false);
        if (_control.joinable()) _control.join();
        _g_commPeriod.store(0);
        SystemStop();
        DisconnectToGripper();

        std::cout << "Disconnected\n"; 
    }
}

// --------------------------------------------------------------------------------

void DGControl::_loop()
{
    auto next = std::chrono::steady_clock::now();

    while(_g_connected.load())
    {
        std::memcpy(_currentPos, _g_gripperData.joint, sizeof(_g_gripperData.joint));   // Запись информации с гриппера

        array2EigenArray(_currentPos, _msgCurrentPos);
        _current_joint_buffer.push(_msgCurrentPos);

        if (_target_joint_buffer.pop(_msgTargetPos))
        {
            eigenArray2Array(_msgTargetPos, _targetPos);
        }

        _updatePos();

        MoveServoJoint(_tempPos); 

        next += std::chrono::microseconds(_g_commPeriod);
        std::this_thread::sleep_until(next);
    }
}

void DGControl::_updatePos()
{
    for (int8_t i = 0; i < MAX_JOINT_COUNT; ++i)
    {
        _tempPos[i] = _tempPos[i] + _deltaPos * handcontrol::sign((_targetPos[i]-_tempPos[i]), _jointThreshold); 
    }
}

// --------------------------------------------------------------------------------

bool DGControl::setTragetPosition(const Eigen::Array<double,MAX_JOINT_COUNT,1> &position)
{
    return _target_joint_buffer.push(position);
}

bool DGControl::getCurrentPosition(Eigen::Array<double,MAX_JOINT_COUNT,1> &position)
{
    return _current_joint_buffer.pop(position);
}

// --------------------------------------------------------------------------------

void handcontrol::eigenArray2Array(const Eigen::Array<double,MAX_JOINT_COUNT,1> &eigen_array, float* array)
{
    for (int8_t i = 0; i < MAX_JOINT_COUNT; ++i)
    {
        array[i] = eigen_array[i];
    }
}

void handcontrol::array2EigenArray(float* array, Eigen::Array<double,MAX_JOINT_COUNT,1> &eigen_array)
{
    for (int8_t i = 0; i < MAX_JOINT_COUNT; ++i)
    {
        eigen_array[i] = array[i];
    }
}

int handcontrol::sign(float a, float threshold)
{
    return (a > std::abs(threshold)) ? 1 : ((a < -std::abs(threshold)) ? -1 : 0);
}