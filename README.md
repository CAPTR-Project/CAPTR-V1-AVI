# CAPTR-V1-AVI
This is the firmware repository for CAPTR-V1 - with the goal of controlled ascent and passive descent.

## Contributing

To make changes, create a branch to work in, and create a Pull Request into `main` when the changes are complete **and have been tested!!!**

PLEASE don't touch other people's branches! (unless you've talked w them)

Use following naming convention:

```
name/branch-name (ex. alex/fsm-skeleton)
```

### Comments

Please write good comments!!! Don't explain what's obvious, but explain the logic behind things when it's unclear. If something is done unorthodoxly, please note it in a comment.

Comment every function with the following format:
```cpp
/**
 * @brief Initializes the IMU sensor on an I2C bus
 * 
 * @param I2CBus The I2C bus to use. Either Wire, Wire1, Wire2
 */
```

### Pull Requests
Ensure your code has been tested to the best of your ability before submitting a PR. Also ensure good documentation and formatting. 

Request another member who is familiar with firmware to review your PR. Ideally, request whoever has the most expertise about your changes.

## Building and Uploading
In VSCode, build the project using PlatformIO.