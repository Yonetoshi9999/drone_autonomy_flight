#!/bin/bash
# Test runner script for aerial photography drone integration tests

set -e  # Exit on error

echo "========================================="
echo "Aerial Photography Drone - Integration Tests"
echo "========================================="
echo ""

# Check if pytest is installed
if ! command -v pytest &> /dev/null; then
    echo "Error: pytest is not installed"
    echo "Install with: pip install pytest pytest-timeout"
    exit 1
fi

# Default: run all tests
TEST_TARGET="${1:-tests/}"

echo "Running tests from: $TEST_TARGET"
echo ""

# Run pytest with various options
pytest "$TEST_TARGET" \
    -v \
    --tb=short \
    --color=yes \
    --strict-markers \
    -W ignore::DeprecationWarning \
    "${@:2}"

echo ""
echo "========================================="
echo "Test run complete!"
echo "========================================="
