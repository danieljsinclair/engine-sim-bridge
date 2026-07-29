// IKeyboardInput.h - Abstract keyboard input reader
// Decouples key reading from terminal specifics for testability.

#ifndef I_KEYBOARD_INPUT_H
#define I_KEYBOARD_INPUT_H

class IKeyboardInput {
public:
    virtual ~IKeyboardInput() = default;

    // Read one key. Returns key code, or -1 if no key available.
    virtual int getKey() = 0;
};

#endif // I_KEYBOARD_INPUT_H
