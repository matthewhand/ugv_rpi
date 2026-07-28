// UGV dashboard UI fix/improve loop.
// One run = N sequential iterations: pick backlog item → implement → verify.
//
// Args:
//   iterations  (int, default 1, max 10) — how many fix cycles this run
//   base_url    (string, default http://127.0.0.1:5000) — for agent notes only
//   dry_run     (bool) — plan only, no code edits
//
// Usage:
//   workflow name=ui-fix-improve args={iterations: 3}
//   validate_only with args={iterations: 1, dry_run: true}

let meta = #{
    name: "ui-fix-improve",
    description: "Iterate UI polish: pick backlog item, implement small fix, verify against screenshots/source",
    when_to_use: "After Playwright UI shots or skeptical UX review; polish Seek/navbar/drawers/Chat without redesign",
    phases: [
        #{ title: "Setup", detail: "load backlog and scope" },
        #{ title: "Iterate", detail: "fix → verify cycles" },
        #{ title: "Summarize", detail: "report done items and remaining" },
    ],
};

let plan_schema = #{
    "type": "object",
    "required": ["item_id", "title", "files", "approach"],
    "properties": #{
        "item_id": #{ "type": "integer" },
        "title": #{ "type": "string" },
        "files": #{ "type": "array", "items": #{ "type": "string" } },
        "approach": #{ "type": "string" },
        "skip_reason": #{ "type": "string" },
    },
};

let fix_schema = #{
    "type": "object",
    "required": ["item_id", "done", "summary", "files_changed"],
    "properties": #{
        "item_id": #{ "type": "integer" },
        "done": #{ "type": "boolean" },
        "summary": #{ "type": "string" },
        "files_changed": #{ "type": "array", "items": #{ "type": "string" } },
        "notes": #{ "type": "string" },
    },
};

let verify_schema = #{
    "type": "object",
    "required": ["ok", "checks"],
    "properties": #{
        "ok": #{ "type": "boolean" },
        "checks": #{ "type": "array", "items": #{ "type": "string" } },
        "regressions": #{ "type": "array", "items": #{ "type": "string" } },
    },
};

// ---- args ----
let iterations = 1;
let base_url = "http://127.0.0.1:5000";
let dry_run = false;
if args != () {
    if args.iterations != () {
        iterations = args.iterations;
        if iterations < 1 { iterations = 1; }
        if iterations > 10 { iterations = 10; }
    }
    if args.base_url != () { base_url = args.base_url; }
    if args.dry_run == true { dry_run = true; }
}

phase("Setup");
log("ui-fix-improve: iterations=" + iterations.to_string() + " dry_run=" + dry_run.to_string());

let setup = agent(
    "Read /home/ws/ugv_rpi/docs/ui-backlog.md and list the next open items (unchecked table rows). "
    + "Confirm templates paths exist under /home/ws/ugv_rpi/templates/. "
    + "Reply with a short plain-text list of open item numbers and titles only.",
    #{ label: "setup-backlog", capability_mode: "read-only" }
);

let results = [];
let i = 0;
while i < iterations {
    let n = i + 1;
    phase("Iterate");
    log("=== iteration " + n.to_string() + "/" + iterations.to_string() + " ===");

    // 1) Pick next open item
    let pick_prompt = "";
    pick_prompt += "You are planning iteration " + n.to_string() + " of a UI fix loop for the UGV dashboard at /home/ws/ugv_rpi.\n";
    pick_prompt += "Read docs/ui-backlog.md. Pick the FIRST open (unchecked) table row by item #.\n";
    pick_prompt += "If none open, set item_id=0, title=\"none\", files=[], approach=\"stop\", skip_reason=\"backlog empty\".\n";
    pick_prompt += "Otherwise fill item_id, title, files (repo-relative), approach (2-4 sentences, concrete CSS/HTML/JS).\n";
    pick_prompt += "Do not implement yet. Use read_file/grep only.";
    let pick = agent(pick_prompt, #{
        label: "pick-" + n.to_string(),
        capability_mode: "read-only",
        output_schema: plan_schema,
    });

    if pick == () || !pick.success || pick.output == () {
        log("pick failed; stopping loop");
        break;
    }
    let plan = pick.output;
    if plan.item_id == 0 || plan.item_id == () {
        log("no more backlog items");
        break;
    }

    log("picked #" + plan.item_id.to_string() + " " + plan.title);

    if dry_run {
        results.push(#{
            iteration: n,
            item_id: plan.item_id,
            title: plan.title,
            done: false,
            dry_run: true,
            approach: plan.approach,
        });
        i += 1;
        continue;
    }

    // 2) Implement
    let fix_prompt = "";
    fix_prompt += "Implement ONE small UI polish item for the Waveshare UGV dashboard.\n";
    fix_prompt += "Repo root: /home/ws/ugv_rpi\n";
    fix_prompt += "Item #" + plan.item_id.to_string() + ": " + plan.title + "\n";
    fix_prompt += "Approach: " + plan.approach + "\n";
    fix_prompt += "Preferred files: " + json_encode(plan.files) + "\n\n";
    fix_prompt += "Rules:\n";
    fix_prompt += "- Smallest change that completes the item; no redesign, no new features.\n";
    fix_prompt += "- Edit only templates/* (html/css/js) and docs/ui-backlog.md unless unavoidable.\n";
    fix_prompt += "- After editing, mark the item done in docs/ui-backlog.md (check the Done column or use [x]).\n";
    fix_prompt += "- Bump cache-bust query on changed CSS/JS in index.html if present (?v=...).\n";
    fix_prompt += "- Do NOT commit or push.\n";
    fix_prompt += "- Use read_file before search_replace; match exact strings.\n";
    fix_prompt += "Return done=true only if you made the functional/visual change.\n";
    let fix = agent(fix_prompt, #{
        label: "fix-" + n.to_string(),
        capability_mode: "read-write",
        output_schema: fix_schema,
    });

    let fix_ok = false;
    let fix_summary = "implement failed";
    let files_changed = [];
    if fix != () && fix.success && fix.output != () {
        fix_ok = fix.output.done == true;
        if fix.output.summary != () { fix_summary = fix.output.summary; }
        if fix.output.files_changed != () { files_changed = fix.output.files_changed; }
    }
    log("fix done=" + fix_ok.to_string() + " " + fix_summary);

    // 3) Verify (static)
    let ver_prompt = "";
    ver_prompt += "Verify UI fix for item #" + plan.item_id.to_string() + " (" + plan.title + ").\n";
    ver_prompt += "Claimed summary: " + fix_summary + "\n";
    ver_prompt += "Files claimed: " + json_encode(files_changed) + "\n";
    ver_prompt += "Read the shipped templates and confirm the change is present (grep/read_file).\n";
    ver_prompt += "Check docs/ui-backlog.md marks the item done if fix succeeded.\n";
    ver_prompt += "Set ok=true only with concrete evidence in checks[]. List regressions if any.\n";
    ver_prompt += "Base URL (optional smoke): " + base_url + " — do not require live browser.\n";
    let ver = agent(ver_prompt, #{
        label: "verify-" + n.to_string(),
        capability_mode: "read-only",
        output_schema: verify_schema,
    });

    let verified = false;
    let checks = [];
    if ver != () && ver.success && ver.output != () {
        verified = ver.output.ok == true;
        if ver.output.checks != () { checks = ver.output.checks; }
    }
    log("verify ok=" + verified.to_string());

    results.push(#{
        iteration: n,
        item_id: plan.item_id,
        title: plan.title,
        done: fix_ok,
        verified: verified,
        summary: fix_summary,
        files_changed: files_changed,
        checks: checks,
    });

    // If implement failed, stop to avoid spinning on same item
    if !fix_ok {
        log("stopping: implement did not complete");
        break;
    }
    i += 1;
}

phase("Summarize");
let report = "# UI fix/improve run\n\n";
report += "iterations_requested: " + iterations.to_string() + "\n";
report += "dry_run: " + dry_run.to_string() + "\n\n";
let done_count = 0;
for r in results {
    report += "## Iter " + r.iteration.to_string() + " — #" + r.item_id.to_string() + " " + r.title + "\n";
    if r.done == true { done_count += 1; report += "- status: DONE\n"; } else { report += "- status: incomplete\n"; }
    if r.verified == true { report += "- verified: yes\n"; } else { report += "- verified: no/unknown\n"; }
    if r.summary != () { report += "- summary: " + r.summary + "\n"; }
    report += "\n";
}
report += "completed_items: " + done_count.to_string() + "\n";

let path = write_scratch_file("ui-fix-improve-report.md", report);
log("report " + path);
complete(#{
    path: path,
    completed: done_count,
    iterations: results.len(),
    results: results,
});
