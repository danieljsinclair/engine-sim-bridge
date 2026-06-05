#ifndef BRAKE_INPUT_H
#define BRAKE_INPUT_H

namespace input {

// Brake with hold-and-decay behaviour. Same pattern as DemoThrottleSource.
// Holds the value for a configurable number of polls (bridging OS key repeat gaps),
// then decays to 0.0 when input stops (key released).
class BrakeInput {
public:
    static constexpr int DEFAULT_HOLD_FRAMES = 8;

    explicit BrakeInput(int holdFrames = DEFAULT_HOLD_FRAMES);

    void setLevel(double level);   // 0.0-1.0, resets hold counter
    double pollLevel();            // holds, then decays to 0.0

private:
    double lastLevel_ = 0.0;
    int holdFrames_;
    int framesSinceSet_ = 0;
};

} // namespace input

#endif
