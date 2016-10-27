#ifndef ROBOT_INFORMATION_H
#define ROBOT_INFORMATION_H

#include <openjaus.h>

namespace openjaus
{
    class OPENJAUS_EXPORT RobotInformation : public model::Message
    {
    public:
        RobotInformation();
        RobotInformation(model::Message *message);
        ~RobotInformation();

        static const uint16_t ID = 0xD000;

        virtual int to(system::Buffer *dst);
        virtual int from(system::Buffer *src);
        virtual int length();
        std::string toXml(unsigned char level = 0) const;
        std::string toString() const;

        OPENJAUS_EXPORT friend std::ostream&
        operator<<(std::ostream &output, const RobotInformation &object);
        OPENJAUS_EXPORT friend std::ostream&
        operator<<(std::ostream &output, const RobotInformation *object);

        std::string getRobotType();
        bool setRobotType(std::string value);

        uint32_t getRobotID();
        bool setRobotID(uint32_t value);

        std::string getRobotName();
        bool setRobotName(std::string value);

        std::string getRobotDescription();
        bool setRobotDescription(std::string value);

        int16_t getLocalizationType();
        bool setLocalizationType(int16_t value);

        float getX();
        bool setX(float value);

        float getY();
        bool setY(float value);

        float getTheta();
        bool setTheta(float value);

    private:
        model::fields::VariableLengthString robotType;
        model::fields::UnsignedInteger robotID;
        model::fields::VariableLengthString robotName;
        model::fields::VariableLengthString robotDescription;
        model::fields::Short localizationType;
        model::fields::Float x;
        model::fields::Float y;
        model::fields::Float theta;
    };
} // namespace openjaus

#endif // ROBOT_INFORMATION_H