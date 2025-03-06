#!/bin/bash
set -e

# Define target directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET_DIR="${SCRIPT_DIR}/include/flatbuffers"

# Create the target directory if it doesn't exist
echo "Creating target directory ${TARGET_DIR}..."
mkdir -p "${TARGET_DIR}"

# Create a temporary directory for cloning the repository
TMP_DIR=$(mktemp -d)
echo "Cloning the Flatbuffers repository..."
git clone --depth 1 https://github.com/google/flatbuffers.git "${TMP_DIR}/flatbuffers"

# Copy all files from the flatbuffers include directory to the target directory
echo "Copying files to ${TARGET_DIR}..."
cp -r "${TMP_DIR}/flatbuffers/include/flatbuffers/"* "${TARGET_DIR}"

# Remove the temporary directory
echo "Cleaning up..."
rm -rf "${TMP_DIR}"

echo "Done. Files have been copied to ${TARGET_DIR}"
