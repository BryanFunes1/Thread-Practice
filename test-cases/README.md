# Runway Assignment Test Cases

This directory contains 10 test cases ranging from simple to very complex aircraft interactions.

## Test Case Overview

### Test 1: Simple Sequential (test01_simple.txt)
- **Complexity:** Easiest
- **Purpose:** Basic runway access with no conflicts
- **Tests:** Sequential commercial aircraft with ample spacing
- **Expected:** Aircraft land one after another smoothly

### Test 2: Runway Capacity (test02_capacity.txt)
- **Complexity:** Medium
- **Purpose:** Verify 2-aircraft runway capacity limit
- **Tests:** Concurrent arrivals exceeding capacity
- **Expected:** Third aircraft waits when runway is full

### Test 3: Commercial/Cargo Separation (test03_separation.txt)
- **Complexity:** Medium
- **Purpose:** Test aircraft type separation rules
- **Tests:** Mixed commercial and cargo arrivals
- **Expected:** Commercial and cargo never on runway together

### Test 4: Direction Switching (test04_direction.txt)
- **Complexity:** Hard 
- **Purpose:** Test runway direction management
- **Tests:** 3-consecutive same-direction limit, direction switches
- **Expected:** Direction switches after 3 consecutive + 5s delay

### Test 5: Controller Breaks (test05_breaks.txt)
- **Complexity:** Hard 
- **Purpose:** Verify controller break after 8 aircraft
- **Tests:** Continuous operations through break threshold
- **Expected:** 9th aircraft waits for 5-second controller break

### Test 6: Emergency Priority (test06_emergency.txt)
- **Complexity:** Hardest 
- **Purpose:** Test emergency aircraft priority handling
- **Tests:** Emergency arrivals during normal operations
- **Expected:** Emergencies get priority, admitted within 30s

### Test 7: Fuel Emergencies (test07_fuel.txt)
- **Complexity:** Hardest 
- **Purpose:** Test fuel emergency escalation
- **Tests:** Long waits causing fuel emergencies (wait > fuel_reserve)
- **Expected:** Fuel-critical aircraft get highest priority

### Test 8: Multiple Constraints (test08_complex.txt)
- **Complexity:** Hardest 
- **Purpose:** Test interaction of multiple rules
- **Tests:** Direction switching + breaks + emergencies combined
- **Expected:** All rules enforced simultaneously without conflicts

### Test 9: Stress Test (test09_stress.txt)
- **Complexity:** Hardest
- **Purpose:** High load with rapid arrivals
- **Tests:** Rapid succession of mixed aircraft types
- **Expected:** No deadlocks, starvation, or synchronization failures

### Test 10: Maximum Complexity (test10_maximum.txt)
- **Complexity:** The most complex 
- **Purpose:** All edge cases and constraints simultaneously
- **Tests:** Long operations, fuel emergencies, breaks, direction switches, priority conflicts
- **Expected:** Perfect synchronization under maximum stress

## Running the Tests

```bash
# Compile the program
make

# Run a specific test
./runway test_cases/test01_simple.txt

# Run all tests
for test in test_cases/test*.txt; do
    echo "Running $test..."
    ./runway "$test"
    echo "---"
done
```

## What Each Test Validates

| Test | Capacity | Separation | Direction | Breaks | Emergency | Fuel | Deadlock |
|------|----------|------------|-----------|--------|-----------|------|----------|
| 01   | ✓        |            |           |        |           |      |          |
| 02   | ✓✓       |            |           |        |           |      |          |
| 03   | ✓        | ✓✓         |           |        |           |      |          |
| 04   | ✓        | ✓          | ✓✓        |        |           |      |          |
| 05   | ✓        | ✓          |           | ✓✓     |           |      |          |
| 06   | ✓        | ✓          | ✓         |        | ✓✓        |      |          |
| 07   | ✓✓       | ✓          | ✓         |        |           | ✓✓   | ✓        |
| 08   | ✓✓       | ✓✓         | ✓✓        | ✓✓     | ✓✓        |      | ✓✓       |
| 09   | ✓✓✓      | ✓✓         | ✓✓        | ✓      | ✓         |      | ✓✓✓      |
| 10   | ✓✓✓      | ✓✓✓        | ✓✓✓       | ✓✓     | ✓✓✓       | ✓✓✓  | ✓✓✓      |

✓ = Basic testing, ✓✓ = Moderate testing, ✓✓✓ = Extensive testing

## Expected Behavior Patterns

### Correct Implementation Should Show:
- Aircraft wait appropriately when runway is full
- Commercial and cargo never appear together
- Direction switches occur with 5-second delays
- Controller takes 5-second breaks after every 8 aircraft
- Emergency aircraft bypass normal queue (but respect capacity)
- Fuel-critical aircraft get absolute priority
- No deadlocks or infinite waits
- Fair scheduling prevents starvation

### Common Bugs These Tests Catch:
- Race conditions in runway access
- Incorrect priority ordering
- Missing direction switch logic
- Controller break not enforced
- Fuel emergency not detected
- Deadlock during direction switches
- Starvation of cargo or commercial aircraft
- Integer overflow in counters
- Missing synchronization primitives

## Debugging Tips

If a test hangs or fails:
1. Check assert statements for which constraint was violated
2. Add debug output showing aircraft states and waiting reasons
3. Verify condition variable signaling is correct
4. Check for missing lock releases
5. Ensure all threads can make progress
6. Verify fuel tracking is working (time-based)
7. Check direction switch logic (runway must be empty)

## File Format

Each line in test files has three numbers:
```
aircraft_type arrival_delay runway_time
```

- `aircraft_type`: 0=Commercial (prefers North), 1=Cargo (prefers South), 2=Emergency (either)
- `arrival_delay`: Seconds since previous aircraft arrival (first aircraft uses 0)
- `runway_time`: Seconds the aircraft needs on the runway
- Fuel reserve: Randomly assigned 20-60 seconds per aircraft at creation time
