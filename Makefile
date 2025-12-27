CC = gcc
CFLAGS = -Wall -Wextra -Werror -std=c99 -pthread
TARGET = runway
SOURCE = runway.c
TEST_DIR = test-cases

.PHONY: all clean test

all: $(TARGET)

$(TARGET): $(SOURCE)
	$(CC) $(CFLAGS) -o $(TARGET) $(SOURCE)

clean:
	rm -f $(TARGET)

test: $(TARGET)
	@echo "Running test cases..."
	@for test_file in $(TEST_DIR)/*.txt; do \
		echo "Testing $$test_file"; \
		./$(TARGET) "$$test_file"; \
		echo ""; \
	done

help:
	@echo "Available targets:"
	@echo "  all     - Build the runway executable"
	@echo "  clean   - Remove compiled files"
	@echo "  test    - Run all test cases"
	@echo "  help    - Show this help message"