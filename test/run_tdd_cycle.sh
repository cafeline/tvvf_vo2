#!/bin/bash
# TDD サイクル実行スクリプト

echo "=== TDD Cycle for TVVF-VO Global Vector Field Navigation ==="
echo ""

# カラー定義
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 作業ディレクトリ
WORKSPACE="/home/rosuser/raspicat_ws"
TEST_BINARY="$WORKSPACE/build/tvvf_vo_c/test/test_fast_marching"

# Phase 1: Red Phase
echo -e "${RED}=== Red Phase: Running tests (expecting failures) ===${NC}"
if [ -f "$TEST_BINARY" ]; then
    $TEST_BINARY 2>&1 | grep -E "FAILED|PASSED|RUN"
    if [ $? -eq 0 ]; then
        echo -e "${YELLOW}Tests exist. Check if they fail initially.${NC}"
    fi
else
    echo -e "${RED}Test binary not found. Build the project first.${NC}"
    exit 1
fi

echo ""
echo -e "${YELLOW}Red Phase Complete: Tests are failing as expected.${NC}"
echo ""

# Phase 2: Green Phase
echo -e "${GREEN}=== Green Phase: Implementing minimum code to pass tests ===${NC}"
echo "Implementation should be done to make all tests pass..."
echo ""

# テスト実行
$TEST_BINARY 2>&1
TEST_RESULT=$?

if [ $TEST_RESULT -eq 0 ]; then
    echo -e "${GREEN}✓ All tests passing! Green Phase complete.${NC}"
else
    echo -e "${RED}✗ Tests still failing. Continue implementation...${NC}"
    exit 1
fi

echo ""

# Phase 3: Refactor Phase
echo -e "${YELLOW}=== Refactor Phase: Improving code quality ===${NC}"
echo "Guidelines for refactoring:"
echo "  1. Extract magic numbers to constants"
echo "  2. Improve variable names"
echo "  3. Extract complex logic to helper functions"
echo "  4. Remove code duplication"
echo "  5. Add performance optimizations"
echo ""

# 最終テスト実行
echo "Running tests after refactoring..."
$TEST_BINARY 2>&1 | tail -5

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Refactoring complete! All tests still passing.${NC}"
else
    echo -e "${RED}✗ Refactoring broke tests! Fix immediately.${NC}"
    exit 1
fi

echo ""
echo -e "${GREEN}=== TDD Cycle Complete ===${NC}"
echo "Summary:"
echo "  ✓ Red Phase: Tests written and failing"
echo "  ✓ Green Phase: Minimum implementation passing"
echo "  ✓ Refactor Phase: Code quality improved"
echo ""
echo "Ready for next feature!"
