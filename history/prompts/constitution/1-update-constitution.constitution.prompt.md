---
id: 1
title: Update Constitution
stage: constitution
date: 2025-12-04
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: Ali Bariz
command: /sp.constitution
labels: ["docs", "constitution"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
---

## Prompt

```text
Project: AI/Spec Driven Development Book on Physical AI & Humanoid Robotics.
Platforms: Docusaurus -> Github Pages
Tools: Spec-Kit Plus + Claude Code
Topic: AI-native software development research for a computer-science academic audience.
Manuscript Goal: Produce a 5,000–7,000-word book with ≥15 sources (≥50% peer-reviewed), APA 7 citations, zero plagiarism, and full fact traceability.



Core Principles
Accuracy: All claims verified against primary or reliable sources.
Clarity: Academic tone for CS readers; readability grade 10–12 (Flesch-Kincaid).
Reproducibility: Every fact must be cited and traceable.
Integrity: 0% plagiarism tolerance; no fabricated or hallucinated citations.
Openness: Book structured as documentation and deployed publicly for review.



Standards
Length: 5,000–7,000 words (book).
Sources: Minimum 15 references; ≥50% must be peer-reviewed (journals/conferences).
Citation Style: APA 7th Edition.
Verification: All facts must pass fact-checking review.
Plagiarism Policy: Must pass academic plagiarism scanners with 0% matches to uncited text.
Format Output: PDF export for submission + Docusaurus website deployment.



Structure Requirements
The book must include: Abstract, Introduction, Core Paradigms, AI Agents in Workflows, Spec-Driven Engineering, Case Studies, Future Directions, Conclusion, References. Each chapter must end with citations where applicable.



Tool Protocol
Constitution is created and validated using Spec-Kit Plus CLI command /sp.constitution.
Writing assistance may use Claude Code, but every generated factual statement must be verified and cited.
Content must follow Docusaurus documentation structure (headers, sidebar-friendly sections).
Deployment must be public on GitHub Pages.



Success Criteria
All claims cited and verifiable.
≥15 sources included with ≥50% peer-reviewed.
Zero plagiarism before submission.
Readability standards met.
Successful deployment to GitHub Pages.



Ownership & Amendments
Authored and owned by Ali Bariz.
Changes must preserve accuracy, reproducibility, and plagiarism-free requirements, tracked via Git.
```

## Response snapshot

```text
The constitution has been updated to version 1.0.0, incorporating all the details provided for the "AI/Spec Driven Development Book on Physical AI & Humanoid Robotics." project.
```

## Outcome

- ✅ Impact: Constitution updated with new project details.
- 🧪 Tests: N/A
- 📁 Files: .specify/memory/constitution.md updated.
- 🔁 Next prompts: Follow-up on updating dependent templates.
- 🧠 Reflection: Successfully updated the constitution based on user's detailed input. Need to ensure dependent templates are updated for consistency.

## Evaluation notes (flywheel)

- Failure modes observed: create-phr.sh script not found.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Ensure create-phr.sh script is available or improve fallback mechanism for PHR creation.
