---
name: falsify
description: Adversarial gate before any "done" or "diagnosed" claim — enumerate the claims being made, attempt to disprove each one against live/real data, and report SURVIVED/REFUTED/UNTESTABLE with evidence. Two modes — change (dev work before/after promote) and hypothesis (investigation RCA before concluding). Run before merging nontrivial changes, before declaring a root cause, and before telling anyone something works.
argument-hint: "[change <PR/diff/description> | hypothesis <statement>]"
allowed-tools: Bash, Read, Grep, Glob
---

# /falsify — try to break it before you believe it

**Why this exists (2026-07-14, Travis):** the hardest problems always show up in the
last mile. The falsification habit has repeatedly out-performed verification-by-
confirmation here — concrete precedents:

- **2026-07-10 LD "vendor outage"**: breadth evidence + vendor status page built a
  HIGH-confidence vendor-outage RCA. Falsification (pull pod logs, look for what the
  hypothesis says should NOT be there) found explicit `401 invalid SDK key` → the real
  cause was an internal bad key, and the original "DO NOT RESTART" guidance would have
  been actively harmful. The alert evidence was real; the conclusion was wrong.
- **PR #56 (change_events)**: all tests green, CI green, deployed — live falsification
  15 minutes later found raw socket events carry no `Text:` label; first two live docs
  were silently wrong.
- **PR #59 (session registry)**: same pattern — the `mongo` heredoc bug made the repair
  script SILENTLY no-op ("repaired: 0" read as "nothing to repair").

The common failure: **evidence that is consistent with your claim is not evidence your
claim is true.** This skill forces the disconfirming test.

## Mode: `change` — before promoting dev work

1. **Enumerate the claims.** Every change implicitly claims things: "X parses all
   message variants", "the endpoint rejects bad tokens", "the backfill covered
   everything", "no behavior change when the flag is off". Write them as a list —
   including the *negative-space* claims (what should NOT happen).
2. **For each claim, derive the cheapest disconfirming probe** — prefer, in order:
   - **live/real data** over fixtures (every parser regression this month was a
     real-data shape no fixture contained: era-specific flattening, Slack truncation,
     linkified markers, missing labels);
   - **the oldest and newest instances** of the data the change touches (formats drift
     — probe both ends of the corpus, not the middle);
   - **absence checks** — query for what should now be impossible (rows still NULL,
     sessions still open, 200s that should be 401s);
   - **the failure path** — feed it garbage, expired tokens, truncated bodies, the
     wrong principal.
3. **Execute the probes** (read-only against prod; full send against dev/VM).
4. **Report the table** and verdict:

   | Claim | Falsification attempt | Result | Evidence |
   |---|---|---|---|
   | … | … | SURVIVED / REFUTED / UNTESTABLE (why) | verbatim output |

   - Any REFUTED → the change is not done; fix and re-run.
   - UNTESTABLE claims must be named and carried as explicit risk, never silently.
5. **Silent-success check** (the PR #59 lesson): any script/migration that reports a
   count must be challenged when that count is suspiciously round, zero, or equal to
   "nothing to do" — prove the tool *can* detect work by giving it work.

## Mode: `hypothesis` — before concluding an investigation

1. State the leading hypothesis in one falsifiable sentence.
2. Generate the **3 strongest rival explanations** (not strawmen — steelman them).
3. For the leading hypothesis AND each rival, answer: *what evidence would I expect to
   find if this were true — and crucially, what should I NOT find?* Check for the
   should-NOT-find items first; they kill hypotheses fastest.
4. Weight discriminating evidence over consistent evidence: the LD incident had a
   mountain of consistent evidence for "vendor outage" and one discriminating fact
   (explicit 401s + a clean vendor status page) that decided it the other way.
5. Re-run `/confidence` after; only then write the RCA. (This formalizes the existing
   rule in [[feedback_confidence_and_falsification]] — that memory's principle, as a
   runnable process.)

## Integration points

- **Dev loop**: step 8 of `docs/dev-loop.md` (Interstellar repo) — mandatory before a
  change is declared done; the live-verification step feeds it.
- **Investigations**: mandatory before Phase-3 documentation when confidence <80 or
  when the RCA drives an action that is expensive/irreversible (fleet restarts,
  customer comms, RCCA publication).
- **Time-box**: 10–20 minutes for changes, longer only when a probe needs prod access
  approval. An unfalsified claim shipped late still beats a refuted claim shipped fast.
