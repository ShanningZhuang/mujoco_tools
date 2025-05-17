# MuJoCo Tools Tests

This directory contains unit and integration tests for the MuJoCo Tools package.

## Running Tests

To run all tests:

```bash
pytest
```

To run a specific test file:

```bash
pytest tests/test_data_processor.py
```

To run a specific test:

```bash
pytest tests/test_player.py::TestMujocoPlayer::test_init_kinematics_mode
```

To see more detailed output:

```bash
pytest -v
```

## Test Coverage

To run tests with coverage report:

```bash
pytest --cov=mujoco_tools
```

## Test Structure

- `conftest.py`: Contains shared fixtures used across tests
- `test_data_processor.py`: Tests for data processing functionality
- `test_player.py`: Tests for the MujocoPlayer class
- `test_recorder.py`: Tests for VideoRecorder and StateRecorder classes
- `test_integration.py`: End-to-end integration tests

## Adding New Tests

1. Create a new test file named `test_<module_name>.py`
2. Use the pytest fixture system for setup and teardown
3. Follow the existing pattern of organizing tests into classes
4. Use descriptive test method names that explain what's being tested 