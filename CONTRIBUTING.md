# Contributing to pico1541-rs

Thank you for your interest in contributing to the pico1541-rs project! This document outlines the process for contributing to the project and provides guidelines to help you get started.

## Code of Conduct

Contributors to this project are expected to adhere to a respectful, inclusive, and collaborative approach. Please be considerate of other contributors and users in all communications.

## Getting Started

1. **Fork the Repository**: Start by forking the repository to your own GitHub account.

2. **Clone Your Fork**: Clone your fork to your local machine.
   ```bash
   git clone https://github.com/YOUR_USERNAME/pico1541-rs.git
   cd pico1541-rs
   ```

3. **Set Up Development Environment**: Ensure you have all the necessary tools installed.  See [BUILD.md](Build.md)

4. **Create a Branch**: Create a branch for your changes.
   ```bash
   git checkout -b feature/your-feature-name
   ```

## Development Guidelines

### Coding Standards

- Follow Rust's official style guidelines
- Use the `rustfmt` tool to format your code: `cargo fmt`
- Use `clippy` to catch common mistakes: `cargo clippy`
- Add comprehensive commenting
- Write tests for new functionality

### Commit Messages

- Use clear, descriptive commit messages
- Begin with a short summary line (50 chars or less)
- Optionally followed by a blank line and a more detailed explanation
- Reference issue numbers if applicable: "Fixes #123" or "Related to #456"

### Testing

Before submitting your code:

1. Make sure it builds without warnings: `cargo build`
2. Run the test suite: `cargo test`
3. Test on actual hardware

## Pull Request Process

1. **Update Your Fork**: Before submitting, make sure your fork is up to date with the main repository.
   ```bash
   git remote add upstream https://github.com/piersfinlayson/pico1541-rs.git
   git fetch upstream
   git merge upstream/main
   ```

2. **Submit Your PR**: Push your changes to your fork and submit a pull request.
   - Provide a clear description of the changes
   - Mention any issues that are addressed by the PR
   - Include screenshots or output examples if relevant
   - Explain what testing you have done

3. **Code Review**: Wait for a project maintainer to review your code.
   - Be responsive to feedback and questions
   - Make requested changes promptly

4. **Merge**: Once approved, a maintainer will merge your PR.

## Reporting Issues

If you find a bug or want to request a feature:

1. Check existing issues to avoid duplicates
2. Use the issue template if available
3. Provide detailed information about the bug or feature request
4. Include steps to reproduce for bugs
5. For hardware issues, specify your exact setup

## Development Roadmap

Current development priorities:

1. Complete xum1541 compatibility mode, first for standard IEC then for addition protocols
2. Implement WiFi interface
3. Add memory caching functionality
4. Develop enhanced error handling

If you're interested in working on any of these areas, please reach out to discuss before starting significant work.

## Contact

For questions or to discuss larger contributions, please contact the project owner at <piers@piers.rocks>.