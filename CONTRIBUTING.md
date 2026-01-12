# Contributing to AeroSimX

Thank you for your interest in contributing to AeroSimX! This document provides guidelines and instructions for contributing.

## 📋 Table of Contents

- [Code of Conduct](#code-of-conduct)
- [Getting Started](#getting-started)
- [Development Setup](#development-setup)
- [Making Changes](#making-changes)
- [Coding Standards](#coding-standards)
- [Testing](#testing)
- [Documentation](#documentation)
- [Pull Request Process](#pull-request-process)

## Code of Conduct

By participating in this project, you agree to abide by our [Code of Conduct](CODE_OF_CONDUCT.md). Please read it before contributing.

## Getting Started

1. **Fork the repository** on GitHub
2. **Clone your fork** locally:
   ```bash
   git clone https://github.com/YOUR_USERNAME/AeroSimX.git
   cd AeroSimX
   ```
3. **Add upstream remote**:
   ```bash
   git remote add upstream https://github.com/ismailtasdelen/AeroSimX.git
   ```

## Development Setup

### Prerequisites

- C++ compiler with C++20 support (GCC 10+, Clang 12+, MSVC 2019+)
- CMake 3.20+
- Python 3.9+
- Git

### Building from Source

```bash
# Create build directory
mkdir build && cd build

# Configure with development options
cmake .. \
    -DCMAKE_BUILD_TYPE=Debug \
    -DBUILD_TESTS=ON \
    -DBUILD_EXAMPLES=ON \
    -DBUILD_PYTHON_BINDINGS=ON

# Build
cmake --build . -j$(nproc)

# Run tests
ctest --output-on-failure
```

### Python Development

```bash
# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install in development mode
pip install -e ".[dev]"

# Install pre-commit hooks
pre-commit install
```

## Making Changes

### Branch Naming

- `feature/` - New features
- `bugfix/` - Bug fixes
- `docs/` - Documentation changes
- `refactor/` - Code refactoring
- `test/` - Test additions/fixes

Example: `feature/add-boat-vehicle`

### Commit Messages

Follow conventional commits format:

```
<type>(<scope>): <description>

[optional body]

[optional footer]
```

Types:
- `feat`: New feature
- `fix`: Bug fix
- `docs`: Documentation
- `refactor`: Code refactoring
- `test`: Tests
- `chore`: Maintenance

Example:
```
feat(sensors): add ultrasonic sensor implementation

- Implement UltrasonicSensor class
- Add configurable range and beam angle
- Include noise model

Closes #123
```

## Coding Standards

### C++ Style

We follow the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html) with modifications:

- **Indentation**: 4 spaces (no tabs)
- **Line length**: 100 characters max
- **Naming**:
  - Classes: `PascalCase`
  - Functions/Methods: `snake_case`
  - Variables: `snake_case`
  - Constants: `UPPER_SNAKE_CASE`
  - Member variables: `trailing_underscore_`

```cpp
class MyClass {
public:
    void do_something();
    
private:
    int member_variable_;
    static constexpr int MAX_VALUE = 100;
};
```

### Python Style

We follow [PEP 8](https://pep8.org/) with these tools:
- **Formatter**: Black
- **Linter**: Ruff
- **Type Checker**: MyPy

```python
def calculate_trajectory(
    start_position: Vector3,
    end_position: Vector3,
    velocity: float = 5.0,
) -> List[Vector3]:
    """Calculate trajectory between two points.
    
    Args:
        start_position: Starting position
        end_position: Target position
        velocity: Travel velocity in m/s
        
    Returns:
        List of waypoints
    """
    pass
```

### Pre-commit Hooks

Run before committing:

```bash
pre-commit run --all-files
```

## Testing

### C++ Tests

```bash
cd build
ctest --output-on-failure

# Run specific test
./tests/cpp/test_physics
```

### Python Tests

```bash
# All tests
pytest tests/python/ -v

# With coverage
pytest --cov=pyaerosimx --cov-report=html

# Specific test file
pytest tests/python/test_client.py -v
```

### Writing Tests

- Place C++ tests in `tests/cpp/`
- Place Python tests in `tests/python/`
- Name test files `test_*.py` or `*_test.cpp`
- Aim for >80% code coverage

## Documentation

### Docstrings

**C++ (Doxygen)**:
```cpp
/**
 * @brief Calculate the distance between two points
 * @param a First point
 * @param b Second point
 * @return Distance in meters
 */
double calculate_distance(const Vector3& a, const Vector3& b);
```

**Python (Google style)**:
```python
def calculate_distance(a: Vector3, b: Vector3) -> float:
    """Calculate the distance between two points.
    
    Args:
        a: First point
        b: Second point
        
    Returns:
        Distance in meters
    """
```

### Building Documentation

```bash
cd docs
pip install -r requirements.txt
make html
```

## Pull Request Process

1. **Update your fork**:
   ```bash
   git fetch upstream
   git rebase upstream/main
   ```

2. **Create a branch**:
   ```bash
   git checkout -b feature/my-feature
   ```

3. **Make changes** and commit

4. **Push to your fork**:
   ```bash
   git push origin feature/my-feature
   ```

5. **Open a Pull Request** on GitHub

### PR Requirements

- [ ] All tests pass
- [ ] Code follows style guidelines
- [ ] Documentation updated (if applicable)
- [ ] Changelog updated (for user-facing changes)
- [ ] No merge conflicts with main

### Review Process

1. Automated CI checks must pass
2. At least one maintainer approval required
3. Address all review comments
4. Squash commits before merge (if requested)

## Questions?

- Open an [Issue](https://github.com/ismailtasdelen/AeroSimX/issues)
- Start a [Discussion](https://github.com/ismailtasdelen/AeroSimX/discussions)

Thank you for contributing! 🚀
