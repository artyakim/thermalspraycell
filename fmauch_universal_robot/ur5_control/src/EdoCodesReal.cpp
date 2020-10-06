#include "EdoCodesReal.h"

EdoCodesReal::EdoCodesReal()
{};

EdoCodesReal::~EdoCodesReal()
{};

std::string EdoCodesReal::getErrorMessages(int8_t error_num)
{
    switch (error_num) {
        case -1:   return "The service is not supported.";
        case -2:   return "The specified RobotNumber is out of range.";
        case -3:   return "Unknown TargetID.";
        case -4:   return "Unknown EventID.";
        case -5:   return "The specified parameter is not found.";
        case -6:   return "The specified tool is not found.";
        case -7:   return "The specified object is not found.";
        case -8:   return "The specified event number is not found.";
        case -9:   return "The specified message number is not found.";
        case -10:  return "The specified joint is not found. No such joint.";
        case -11:  return "No more parameters. End of file.";
        case -12:  return "The specified input format is not supported.";
        case -13:  return "The specified output format is not supported.";
        case -14:  return "The specified storage is not supported.";
        case -15:  return "The specified accuracy type is not supported.";
        case -16:  return "The specified event type is not supported.";
        case -17:  return "The specified motion type is not supported.";
        case -18:  return "The specified target type is not supported.";
        case -19:  return "The specified orientation interpolation mode is not supported.";
        case -20:  return "The specified dominant interpolation space is not supported.";
        case -21:  return "The specified parameter can not be modified (no write access).";
        case -22:  return "Wrong number of parameters for ORL service.";
        case -23:  return "The format of the input data is wrong.";
        case -24:  return "The format of the configuration string is wrong.";
        case -25:  return "The motion is not possible in the specified time.";
        case -26:  return "The specified parameter value is out of range.";
        case -27:  return "The specified time is out of range.";
        case -28:  return "The specified distance is out of range.";
        case -29:  return "The specified encoder value is out of range.";
        case -30:  return "The specified speed value is out of range.";
        case -31:  return "The specified acceleration value is out of range.";
        case -32:  return "The specified correction type is out of range.";
        case -33:  return "String is too long.";
        case -34:  return "Error in matrix. Incomplete matrix.";
        case -35:  return "Cartesian position expected.";
        case -36:  return "Joint position expected.";
        case -37:  return "No robot between object and tool.";
        case -38:  return "Too many events defined.";
        case -39:  return "The specified condition is not valid.";
        case -40:  return "There are no events.";
        case -41:  return "There are no messages.";
        case -42:  return "No target set.";
        case -43:  return "Initial position not set.";
        case -44:  return "Tracking is not set.";
        case -45:  return "Too many conveyors set.";
        case -46:  return "Not able to create/update file.";
        case -47:  return "Illegal use of service. Can't do that.";
        case -48:  return "Memory problem.";
        case -49:  return "Internal software error.";
        case -50:  return "Other error.";
        case -51:  return "No solution is found. One joint is out of range.";
        case -52:  return "Cartesian position is out of work range.";
        case -53:  return "Too many controllers.";
        case -54:  return "STOP_MOTION is unsuccessful. No motion in progress.";
        case -55:  return "The service is not performed. Motion in progress.";
        case -56:  return "Initialization is not performed. Machine data file is not found.";
        case -57:  return "The specified frame is not found.";
        case -58:  return "Service cannot initialize another instance of a robot.";
        case -59:  return "The specified position is singular.";
        case -60:  return "The specified accelerationn type is not supported.";
        case -61:  return "No more frames. End of file.";
        case -62:  return "The specified frame can not be modified (no write access).";
        case -63:  return "Unsupported rotation axis.";
        case -64:  return "Machine data file not found.";
        case -65:  return "Initialization not performed because of not accepted version number.";
        case -66:  return "Reset level not supported.";
        case -67:  return "The output-block is too short.";
        case -68:  return "Fatal error. Stopped calculating (for GET_NEXT_STEP only).";
        case -69:  return "The parameter ManipulatorType is not supported.";
        case -70:  return "The specified ManipulatorType is not valid.";
        case -71:  return "Position not stored; target buffer is full.";
        case -72:  return "The list of services to debug is full.";
        case -73:  return "The FirstNext mechanism is not supported.";
        case -74:  return "Position overspecified.";
        case -75:  return "Position underspecified.";
        case -76:  return "Can't move; incomplete or inconsistent motion specification.";
        case -77:  return "Input block too short.";
        case -78:  return "Not ready to receive targets.";
        case -79:  return "The specified position is not acceptable.";
        case -80:  return "The specified conveyor is not found.";
        case -81:  return "Service not applicable for this robot type.";
        case -82:  return "The specified group is not found.";
        case -83:  return "The specified jerk value is out of range.";
        case -84:  return "The specified jerk type is out of range.";
        case -85:  return "RobotPathName is write protected.";
        case -86:  return "The current working directory is write protected.";
        case -87:  return "The specified EventID is already in use.";
        case -88:  return "The specified time compensation is not supported.";
        case -89:  return "The same controller has been already initialized.";
        case -90:  return "The controller can't be destroyed.";
        case -91:  return "The controller doesn't exist.";
        case -92:  return "Powerlink Communication not active.";
        case -93:  return "Request Joints Mask not available.";
        case -94:  return "Missing information about some joints.";
        case -95:  return "The function is not available in the current Open Modality .";
        case -96:  return "Requested Values are invalid.";
        case -97:  return "ORL not initialized.";
        case -98:  return "ItDM not available.";
        case -99:  return "Unknown Error.";
        case -100: return "Unknown Error.";
        case -101: return "tipo movimento non riconosciuto";
        case -102: return "Motion not ready";
        case -103: return "Movement was not executed";
        case -104: return "Errore Generico";
        default:   return "This code does not exist !!!";
    }
}