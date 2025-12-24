MuJoCo Pendulum PD

This repository contains a simple MuJoCo pendulum example (`src/main.c`) with a PD controller.

How to use

1. Build (from workspace root):
   mkdir -p build && cd build
   cmake ..
   make

2. Run (example):
   ./pendulum pendulum.xml

Notes
- Ensure MuJoCo and GLFW are installed and accessible to the compiler.
- To enable IntelliSense exact include paths, generate `compile_commands.json` by configuring CMake with `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON`.

Pushing to GitHub

If you want me to add a remote and push, provide the GitHub repository URL (HTTPS or SSH). I will run the `git remote add` and `git push` commands for you, or you can run them yourself:

    git remote add origin <your-repo-url>
    git branch -M main
    git push -u origin main
