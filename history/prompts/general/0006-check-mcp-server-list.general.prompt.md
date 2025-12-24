---
id: 0006
title: Check MCP Server List
stage: general
date: 2025-12-23
surface: agent
model: claude-3-5-sonnet-20241022
feature: none
branch: main
user: user
command: check mcp server list in this project
labels: ["mcp", "configuration"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - backend/uv.lock
tests:
 - null
---

## Prompt

check mcp server list in this project

## Response snapshot

Based on the available tools and project files, here are the MCP servers identified:

**Active MCP Servers (available to the agent):**
1.  **zencoder-server**: Provides IDE integration tools (`get_diagnostics`, `get_open_editors`, `ask_questions`).
2.  **zencoder-rag-mcp**: Provides web search and content fetching tools (`web_search`, `fetch_webpage`).

**Project Dependencies:**
- The `backend` module has a dependency on the `mcp` Python package (found in `backend/uv.lock`), suggesting the project may implement or consume MCP servers programmatically.
- Prompt history references a `context7` MCP server, though it is not currently exposed as a tool to this agent.

## Outcome

- ✅ Impact: Identified active MCP servers and project dependencies.
- 🧪 Tests: N/A
- 📁 Files: Checked backend/uv.lock
- 🔁 Next prompts: N/A
- 🧠 Reflection: The user asked for MCP servers. I identified the ones I'm using via tool definitions and also checked the codebase for MCP dependencies.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
