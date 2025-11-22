#!/bin/bash
# ESP-IDF Environment Setup Script
# Source this file to set up ESP-IDF environment: source setup_esp_idf.sh

# Common ESP-IDF installation locations
ESP_IDF_PATHS=(
    "$HOME/esp/esp-idf"
    "$HOME/esp-idf"
    "/opt/esp/idf"
    "/usr/local/esp-idf"
)

# Check if IDF_PATH is already set
if [ -n "$IDF_PATH" ]; then
    echo "IDF_PATH is already set to: $IDF_PATH"
    if [ -f "$IDF_PATH/export.sh" ]; then
        echo "Sourcing ESP-IDF environment from: $IDF_PATH"
        . "$IDF_PATH/export.sh"
        echo "ESP-IDF environment ready!"
        echo "IDF_PATH: $IDF_PATH"
        echo "Python: $(which python)"
        echo "idf.py: $(which idf.py)"
        return 0 2>/dev/null || exit 0
    else
        echo "Warning: IDF_PATH is set but export.sh not found!"
    fi
fi

# Try to find ESP-IDF in common locations
for path in "${ESP_IDF_PATHS[@]}"; do
    if [ -f "$path/export.sh" ]; then
        echo "Found ESP-IDF at: $path"
        export IDF_PATH="$path"
        . "$path/export.sh"
        echo "ESP-IDF environment ready!"
        echo "IDF_PATH: $IDF_PATH"
        echo "Python: $(which python)"
        echo "idf.py: $(which idf.py)"
        return 0 2>/dev/null || exit 0
    fi
done

# If not found, check if zip file exists and needs extraction
if [ -f "$HOME/esp/esp-idf-v5.5.1.zip" ]; then
    echo "ESP-IDF zip file found at: $HOME/esp/esp-idf-v5.5.1.zip"
    echo ""
    echo "To set up ESP-IDF:"
    echo "1. Extract the zip file:"
    echo "   cd ~/esp && unzip esp-idf-v5.5.1.zip"
    echo ""
    echo "2. Install ESP-IDF tools:"
    echo "   cd ~/esp/esp-idf && ./install.sh esp32c6"
    echo ""
    echo "3. Source this script again:"
    echo "   source setup_esp_idf.sh"
    echo ""
    echo "Or manually source:"
    echo "   . ~/esp/esp-idf/export.sh"
    return 1 2>/dev/null || exit 1
fi

echo "ESP-IDF not found in common locations."
echo ""
echo "To install ESP-IDF:"
echo "1. Clone ESP-IDF:"
echo "   cd ~/esp && git clone --recursive https://github.com/espressif/esp-idf.git -b v5.5.1"
echo ""
echo "2. Install tools:"
echo "   cd ~/esp/esp-idf && ./install.sh esp32c6"
echo ""
echo "3. Source this script:"
echo "   source setup_esp_idf.sh"
return 1 2>/dev/null || exit 1

