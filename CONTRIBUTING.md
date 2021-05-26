# Contribution Guidelines

Thank you for your interest in contributing to Unity Robotics! To facilitate your
contributions, we've outlined a brief set of guidelines to ensure that your extensions
can be easily integrated.

## Communication

First, please read through our
[code of conduct](CODE_OF_CONDUCT.md),
as we expect all our contributors to follow it.

Second, before starting on a project that you intend to contribute to any of our
Unity Robotics packages or tutorials, we **strongly** recommend posting on the repository's
[Issues page](https://github.com/Unity-Technologies/Unity-Robotics-Hub/issues) and
briefly outlining the changes you plan to make. This will enable us to provide
some context that may be helpful for you. This could range from advice and
feedback on how to optimally perform your changes or reasons for not doing it.

## Git Branches

The `main` branch corresponds to the most recent stable version of the project. The `dev` branch
contains changes that are staged to be merged into `main` as the team sees fit.

When contributing to the project, please make sure that your Pull Request (PR)
does the following:

- Is up-to-date with and targets the `dev` branch
- Contains a detailed description of the changes performed
- Has corresponding changes to documentation, unit tests and sample environments (if
  applicable)
- Contains a summary of the tests performed to validate your changes
- Links to issue numbers that the PR resolves (if any)

<!-- ## Continuous Integration (CI)

We run continuous integration on all PRs; all tests must be passing before the PR is merged. -->

## Code style

All Python code should follow the [PEP 8 style guidelines](https://pep8.org/).

All C# code should follow the [Microsoft C# Coding Conventions](https://docs.microsoft.com/en-us/dotnet/csharp/programming-guide/inside-a-program/coding-conventions).
Additionally, the [Unity Coding package](https://docs.unity3d.com/Packages/com.unity.coding@0.1/manual/index.html)
can be used to format, encode, and lint your code according to the standard Unity
development conventions. Be aware that these Unity conventions will supersede the
Microsoft C# Coding Conventions where applicable.

Please note that even if the code you are changing does not adhere to these guidelines,
we expect your submissions to follow these conventions.

## Contributor License Agreements

When you open a pull request, you will be asked to acknowledge our Contributor
License Agreement. We allow both individual contributions and contributions made
on behalf of companies. We use an open source tool called CLA assistant. If you
have any questions on our CLA, please
[submit an issue](https://github.com/Unity-Technologies/Unity-Robotics-Hub/issues) or
email us at [unity-robotics@unity3d.com](mailto:unity-robotics@unity3d.com).

## Contribution review

Once you have a change ready following the above ground rules, simply make a
pull request in GitHub.