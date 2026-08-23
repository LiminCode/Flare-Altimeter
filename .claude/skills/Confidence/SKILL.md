---
name: confidence
description: Rate confidence in the current investigation from 1-100 with reasoning
argument-hint: "[claim or conclusion to evaluate]"
---

# Investigation Confidence Score

Assess your confidence in the current investigation's root cause, findings, and recommendations. Produce a score from 1–100 and explain it.

## Optional Context

```
$ARGUMENTS
```

If `$ARGUMENTS` is provided, treat it as the specific claim or conclusion to evaluate (e.g., "the node failure caused the kafka crash"). Otherwise, evaluate overall confidence in the active investigation.

## Scoring Methodology

Work through each factor. Assign a sub-score per factor, then derive the overall score.

### Factor 1: Root Cause Clarity (0–25 pts)
- **25**: Single clear root cause, confirmed by direct evidence (e.g., OOMKill event + df output agree)
- **15–24**: Root cause identified, supported by strong circumstantial evidence
- **8–14**: Probable root cause, one or more supporting indicators but not definitively confirmed
- **1–7**: Hypothesis only — no direct evidence, working on inference
- **0**: Root cause unknown

### Factor 2: Evidence Breadth (0–20 pts)
- **20**: ≥4 independent sources corroborate (e.g., kubectl events + logs + Slack + Jira)
- **13–19**: 3 sources checked, consistent findings
- **7–12**: 2 sources, or 3+ sources with partial gaps
- **1–6**: Single source only
- **0**: No evidence collected

### Factor 3: Evidence Quality (0–20 pts)
- **20**: All key commands returned clear, unambiguous output; no timeouts, errors, or missing data
- **13–19**: Most outputs clean; minor gaps or one ambiguous result
- **7–12**: Some commands failed or returned partial data (timeouts, NotReady nodes, missing logs)
- **1–6**: Significant data gaps — key commands couldn't run or returned errors
- **0**: No command outputs obtained

### Factor 4: Timeline Coherence (0–15 pts)
- **15**: Complete causal chain reconstructed with timestamps; events fit together cleanly
- **10–14**: Timeline mostly complete; minor gaps or one unexplained jump
- **5–9**: Partial timeline; some events inferred rather than confirmed
- **1–4**: Fragmented timeline; major gaps between trigger and symptom
- **0**: No timeline established

### Factor 5: Alternative Hypotheses Ruled Out (0–10 pts)
- **10**: Competing explanations explicitly considered and eliminated with evidence
- **6–9**: One or two alternatives briefly considered
- **3–5**: No competing hypotheses addressed, but the evidence strongly points one way
- **0–2**: Other explanations remain viable and unaddressed

### Factor 6: Fix Verification (0–10 pts)
- **10**: Fix applied and confirmed working (metric dropped, alert resolved, pod Running)
- **6–9**: Fix applied, partial confirmation (pod starting, waiting on propagation)
- **3–5**: Fix recommended but not yet applied
- **0–2**: No fix attempted or fix failed

## Output Format

Respond with exactly this structure — no preamble, no trailing commentary:

---

## Confidence: [SCORE]/100

**[One-line verdict]** — e.g., "Strong confidence — root cause confirmed by two independent data sources with successful remediation."

### Score Breakdown

| Factor | Score | Max | Notes |
|--------|-------|-----|-------|
| Root Cause Clarity | X | 25 | [one phrase] |
| Evidence Breadth | X | 20 | [one phrase] |
| Evidence Quality | X | 20 | [one phrase] |
| Timeline Coherence | X | 15 | [one phrase] |
| Alternatives Ruled Out | X | 10 | [one phrase] |
| Fix Verification | X | 10 | [one phrase] |
| **Total** | **X** | **100** | |

### Why Not Higher
[1–3 specific things that would increase this score — missing data, unverified steps, open questions. If 95+, state "None — investigation is as complete as the situation allows."]

### Why Not Lower
[1–2 things that anchor the score where it is — the strongest pieces of evidence or confirmation steps completed.]

---
