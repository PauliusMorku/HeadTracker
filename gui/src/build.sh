#!/bin/bash

# Build script for HeadTracker Qt GUI on macOS
# This script handles the AGL framework compatibility issue

echo "Building HeadTracker Qt GUI..."

# Generate Makefile
qmake HeadTracker.pro

# Remove deprecated AGL framework reference
sed -i '' 's/-framework AGL //' Makefile

# Build the application
make -j4

echo "Build complete!"
echo "Executable: HeadTracker.app/Contents/MacOS/HeadTracker"