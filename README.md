# Physical AI & Humanoid Robotics Textbook

This educational website is built using [Docusaurus](https://docusaurus.io/), a modern static website generator designed for documentation and textbooks.

## Installation

```bash
yarn
```

## Local Development

```bash
yarn start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

## Build

```bash
yarn build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

## Deployment

Using SSH:

```bash
USE_SSH=true yarn deploy
```

Not using SSH:

```bash
GIT_USER=<Your GitHub username> yarn deploy
```

If you are using GitHub pages for hosting, this command is a convenient way to build the website and push to the `gh-pages` branch.

## Project Constitution

This project follows the principles outlined in `.specify/memory/constitution.md`:

- **Clarity First**: Crystal-clear explanations for beginners in robotics
- **Encouraging Tone**: Supportive mentor voice throughout
- **Hands-on Mandate**: Complete, copy-paste-ready code examples
- **Zero Fluff**: Every word must teach - no filler content
- **Complete Syllabus Coverage**: 100% coverage of required content
- **Verbatim Requirements**: Hardware tables reproduced exactly from syllabus
