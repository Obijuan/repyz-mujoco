#!/bin/bash

# Virtual environment setup script for REPYZ Minicube simulation

echo "Creating Python virtual environment in the 'venv' folder..."
python3 -m venv venv

echo "Activating the virtual environment temporarily..."
source venv/bin/activate

echo "Updating pip to the latest version..."
pip install --upgrade pip

echo "Installing required dependencies..."
# Exact versions detected on the system are installed to ensure compatibility
pip install mujoco==3.13.0 numpy==2.4.3 viser mjviser

echo "========================================================="
echo "Virtual environment installed and configured successfully!"
echo "Remember that to use the simulator you must activate the environment"
echo "each time you open a new terminal by running:"
echo ""
echo "    source venv/bin/activate"
echo "========================================================="
