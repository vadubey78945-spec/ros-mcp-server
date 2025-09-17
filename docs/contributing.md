# Contributing to ROS-MCP-Server

---

## 📌 Ways to Contribute

- **Report issues**: Bug reports, documentation gaps, or feature requests.  
- **Improve documentation**: Clarify installation steps, add tutorials, or share demos.  
- **Submit code**: Bug fixes, new features, refactors, or tests.  
- **Share use cases**: Show how you use ROS-MCP with your robot.  
- **Community support**: Answer GitHub Discussions or ROS Discourse questions.  

---

## 🔀 How to Pull-request

1. **Fork the repository:** Click the "Fork" button in the top right corner of the GitHub page.

2. **Create a new branch:**  Create a separate branch for your changes to keep them isolated from the main project.

   ```bash
   git checkout -b "your_branch_name"
   ```

3. **Make your changes:** Edit files with your additions or corrections.  Please follow the existing format and style.  When adding a new function, make sure to include:

    * The function name and format.
    * A brief description of the function's functionality.
    * Any parameters the function takes.
    * The return type of the function.

4. **Commit your changes:** Commit your changes with a clear and concise message explaining what you've done.

   ```bash
   git commit -m "your_commit_message"
   ```

5. **Push your branch:** Push your branch to your forked repository.

   ```bash
   git push origin "your_branch_name"
   ```

6. **Create a pull request:** Navigate to the original repository and click the "New pull request" button. Select your fork and the branch you pushed. **Important:** Target the `develop` branch (not `main`) for your pull request. Provide a clear title and description of your changes.

7. **Review and merge:** Your pull request will be reviewed by the maintainers. They may suggest changes or ask for clarification. Once approved, your changes will be merged into the main repository.

Thank you for contributing!

---

## Style & CI
We use **Ruff** for both linting and formatting. CI requires:
- `ruff format --check .`
- `ruff check .`

**Local setup**
- Option A (recommended): `pre-commit install` (auto-fixes on commit)
- Option B (manual): `ruff format . && ruff check --fix .`

**Using Black locally?**
- That's fine. Please align with:
  - `line-length = 100`, target `py310`
  - Avoid `--preview`, `--skip-string-normalization`, `--skip-magic-trailing-comma`
- CI uses Ruff as the final arbiter; run `ruff format .` before pushing.

---

## License

This project is licensed under the **Apache License 2.0**. By contributing to this project, you agree that your contributions will be licensed under the same license.

**Key points for contributors:**
- Your contributions will be licensed under Apache 2.0
- You retain copyright to your contributions
- You grant the project a perpetual, worldwide, non-exclusive license to use your contributions
- No additional legal agreements required for standard contributions

For the full license text, see [LICENSE](../LICENSE) in the project root.
