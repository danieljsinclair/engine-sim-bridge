#ifndef BRAKE_INPUT_H
#define BRAKE_INPUT_H

namespace input {

// Brake pressure tracker. Holds the last set level until explicitly changed.
// No decay — brake is either applied (pressure > 0) or released (0).
class BrakeInput {
public:
    BrakeInput() = default;

    void setLevel(double level);   // 0.0-1.0
    double pollLevel() const;      // returns current level (unchanged until setLevel called)

private:
    double level_ = 0.0;
};

} // namespace input

#endif
