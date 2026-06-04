#ifndef IGNITION_INPUT_H
#define IGNITION_INPUT_H

namespace input {

// Manages ignition state. Pure state object — consumer calls setOn() to control.
class IgnitionInput {
public:
    IgnitionInput();

    void setOn(bool on);
    bool isOn() const;
    void toggle();

private:
    bool ignitionOn_;
};

} // namespace input

#endif
