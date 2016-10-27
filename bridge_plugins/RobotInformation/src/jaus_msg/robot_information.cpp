#include "jaus_msg/robot_information.h"

using namespace openjaus;

RobotInformation::RobotInformation() :
    model::Message(),
    robotType(),
    robotID(),
    robotName(),
    robotDescription(),
    localizationType(),
    x(),
    y(),
    theta()
{
    this->id = RobotInformation::ID;
    setType(transport::JAUS_MESSAGE);

    robotType.setName("RobotType");
    robotType.setSizeType(model::fields::UNSIGNED_SHORT);
    robotType.setValue("default");

    robotID.setName("RobotID");
    robotID.setValue(0);

    robotName.setName("RobotName");
    robotName.setSizeType(model::fields::UNSIGNED_SHORT);
    robotName.setValue("default");

    robotDescription.setName("RobotDescription");
    robotDescription.setSizeType(model::fields::UNSIGNED_LONG);
    robotDescription.setValue("default");

    localizationType.setName("LocalizationType");
    localizationType.setValue(0);

    x.setName("X");
    x.setValue(0);

    y.setName("Y");
    y.setValue(0);

    theta.setName("Theta");
    theta.setValue(0);
}

RobotInformation::RobotInformation(model::Message *message) :
    model::Message(message),
    robotType(),
    robotID(),
    robotName(),
    robotDescription(),
    localizationType(),
    x(),
    y(),
    theta()
{
    this->id = RobotInformation::ID;
    setType(transport::JAUS_MESSAGE);

    robotType.setName("RobotType");
    robotType.setSizeType(model::fields::UNSIGNED_SHORT);
    robotType.setValue("default");

    robotID.setName("RobotID");
    robotID.setValue(0);

    robotName.setName("RobotName");
    robotName.setSizeType(model::fields::UNSIGNED_SHORT);
    robotName.setValue("default");

    robotDescription.setName("RobotDescription");
    robotDescription.setSizeType(model::fields::UNSIGNED_LONG);
    robotDescription.setValue("default");

    localizationType.setName("LocalizationType");
    localizationType.setValue(0);

    x.setName("X");
    x.setValue(0);

    y.setName("Y");
    y.setValue(0);

    theta.setName("Theta");
    theta.setValue(0);

    system::Buffer *payloadBuffer = dynamic_cast<system::Buffer*>
            (message->getPayload());

    if (payloadBuffer)
    {
        this->from(payloadBuffer);
        payloadBuffer->reset();
    }
}

RobotInformation::~RobotInformation()
{
}

std::string RobotInformation::getRobotType()
{
    return this->robotType.getValue();
}

bool RobotInformation::setRobotType(std::string value)
{
    return this->robotType.setValue(value);
}

uint32_t RobotInformation::getRobotID()
{
    return this->robotID.getValue();
}

bool RobotInformation::setRobotID(uint32_t value)
{
    return this->robotID.setValue(value);
}

std::string RobotInformation::getRobotName()
{
    return this->robotName.getValue();
}

bool RobotInformation::setRobotName(std::string value)
{
    return this->robotName.setValue(value);
}

std::string RobotInformation::getRobotDescription()
{
    return this->robotDescription.getValue();
}

bool RobotInformation::setRobotDescription(std::string value)
{
    return this->robotDescription.setValue(value);
}

int16_t RobotInformation::getLocalizationType()
{
    return this->localizationType.getValue();
}

bool RobotInformation::setLocalizationType(int16_t value)
{
    return this->localizationType.setValue(value);
}

float RobotInformation::getX()
{
    return this->x.getValue();
}

bool RobotInformation::setX(float value)
{
    return this->x.setValue(value);
}

float RobotInformation::getY()
{
    return this->y.getValue();
}

bool RobotInformation::setY(float value)
{
    return this->y.setValue(value);
}

float RobotInformation::getTheta()
{
    return this->theta.getValue();
}

bool RobotInformation::setTheta(float value)
{
    return this->theta.setValue(value);
}

int RobotInformation::to(system::Buffer *dst)
{
    int byteSize = dst->pack(static_cast<uint16>(this->id & 0xFFFF));
    byteSize += dst->pack(robotType);
    byteSize += dst->pack(robotID);
    byteSize += dst->pack(robotName);
    byteSize += dst->pack(robotDescription);
    byteSize += dst->pack(localizationType);
    byteSize += dst->pack(x);
    byteSize += dst->pack(y);
    byteSize += dst->pack(theta);
    return byteSize;
}

int RobotInformation::from(system::Buffer *src)
{
    uint16 temp;
    int byteSize = src->unpack(temp);
    this->id = temp;
    byteSize += src->unpack(robotType);
    byteSize += src->unpack(robotID);
    byteSize += src->unpack(robotName);
    byteSize += src->unpack(robotDescription);
    byteSize += src->unpack(localizationType);
    byteSize += src->unpack(x);
    byteSize += src->unpack(y);
    byteSize += src->unpack(theta);
    return byteSize;
}

int RobotInformation::length()
{
    int length = 0;
    length += sizeof(uint16_t);
    length += robotType.length();
    length += robotID.length();
    length += robotName.length();
    length += robotDescription.length();
    length += localizationType.length();
    length += x.length();
    length += y.length();
    length += theta.length();
    return length;
}

std::string RobotInformation::toString() const
{
    return std::string("RobotInformation [0xD000]");
}

std::ostream& operator<<(std::ostream& output, const RobotInformation& object)
{
    output << object.toString();
    return output;
}

std::ostream& operator<<(std::ostream& output, const RobotInformation* object)
{
    output << object->toString();
    return output;
}

std::string RobotInformation::toXml(unsigned char level) const
{
    std::ostringstream prefix;
    for (unsigned char i = 0; i < level; i++)
    {
        prefix << "\t";
    }

    std::ostringstream oss;
    oss << prefix.str() << "<Message name=\"RobotInformation\"";
    oss << " id=\"0xD000\" >\n";
    oss << robotType.toXml(level + 1);
    oss << robotID.toXml(level + 1);
    oss << robotName.toXml(level + 1);
    oss << robotDescription.toXml(level + 1);
    oss << localizationType.toXml(level + 1);
    oss << x.toXml(level + 1);
    oss << y.toXml(level + 1);
    oss << theta.toXml(level + 1);
    oss << prefix.str() << "</Message>\n";
    return oss.str();
}