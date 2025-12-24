---
id: 0007
title: Query GitHub MCP
stage: general
date: 2025-12-23
surface: agent
model: claude-3-5-sonnet-20241022
feature: none
branch: main
user: user
command: guthub mcp
labels: ["mcp", "github", "info"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - null
tests:
 - null
---

## Prompt

guthub mcp

## Response snapshot

I searched the project and did not find a GitHub MCP server configured.

However, there is an **official GitHub MCP Server** available (https://github.com/modelcontextprotocol/servers/tree/main/src/github) that allows agents to:
- Read repositories
- Create pull requests
- Manage issues
- Search code

If you wish to use it, you would typically configure it in your MCP client settings (e.g., `claude_desktop_config.json` or similar) with your GitHub Personal Access Token.

## Outcome

- ✅ Impact: Clarified that GitHub MCP is not active in project but exists externally.
- 🧪 Tests: N/A
- 📁 Files: N/A
- 🔁 Next prompts: N/A
- 🧠 Reflection: User likely wanted to know if it was available or how to get it. Provided context from web search.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
