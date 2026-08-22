# Seek Navigation: Old vs New Implementation

## OLD CODE (1ce43d7 and earlier) - DEPRECATED

### Image Handling
- **Stitched Panorama**: `_seek_unified_llm_analysis` receives LEFT/CENTRE/RIGHT stitched into one wide image
- **Front is 1/3 of stitch**: Centre view is only the MIDDLE THIRD of the panorama
- **Not full-resolution**: Front view is compressed as part of larger stitch
- **Bug impact**: When L/R captures are front frames (settle bug), panorama is "three fronts" → useless

### LLM Frequency
- **Sparse**: LLM called only on step 1 and every `llm_nav_interval` (default 10)
- **Most steps heuristic-only**: No LLM, no images analyzed
- **Heuristic drives blind**: Can't distinguish cabinet from sofa/edge-dense scenes

## NEW CODE (1be6eef onwards) - CURRENT

### Image Handling
```python
# _seek_analyze_front_view (line 7893)
# Sends SINGLE front JPEG with clear labeling:
"This is the robot's FRONT view (camera 0° straight ahead — drive direction)."

# _seek_analyze_side_view (line 7964)  
# Sends SINGLE side JPEG with clear labeling:
"This is the robot's LEFT view (camera -135°)"
"This is the robot's RIGHT view (camera +135°)"
```

### LLM Frequency
- **EVERY step**: Front view analyzed via LLM on EVERY iteration
- **Side views when blocked**: Left and right analyzed individually when front blocked
- **Forced JSON schemas**: No prose, reliable yes/no answers
- **Full-resolution individual JPEGs**: Each view is complete, not 1/3 of a stitch

## Key Differences

| Aspect | Old (≤1ce43d7) | New (≥1be6eef) |
|--------|----------------|----------------|
| **Front image** | 1/3 of panorama stitch | Full-resolution JPEG |
| **LLM calls** | Step 1, 10, 20, 30... | EVERY step |
| **Side images** | 1/3 of panorama stitch | Individual full JPEGs |
| **Labeling** | "LEFT/CENTRE/RIGHT thirds" | "FRONT", "LEFT at -135°", "RIGHT at +135°" |
| **Bug impact** | Three fronts = garbage | Each view independent |
| **Heuristic** | Drives blind most steps | Always has LLM front analysis |

## Example Step Sequence (New Code)

**Step 1:**
1. Pan to 0° → wait settle → capture `front.jpg`
2. LLM analyzes `front.jpg`: "FRONT view, camera 0°"
3. Result: `clear_forward_little=no, clear_forward_lot=no, subject=no`
4. Front blocked → scan sides

**Step 1 (continued - sides):**
5. Pan to -135° → wait settle → capture `left.jpg`
6. LLM analyzes `left.jpg`: "LEFT view, camera -135°"
7. Result: `direction_clear=yes`
8. Rotate left, go to Step 2

**Step 2:**
9. Pan to 0° → wait settle → capture NEW `front.jpg`
10. LLM analyzes NEW front: fresh analysis
11. Continue...

## Settle Bug Impact

**Before settle fix (rear = front frames):**
- Old code: Panorama = three identical fronts → LLM confused
- New code: Front OK, but left/right also fronts → LLM says both blocked (correct, since they show front!)

**After settle fix (commits d09c477, 26d264b, 78c7e02):**
- All views wait proper slew time before capture
- Front is front, left is actual -135° rear-left, right is actual +135° rear-right
- LLM sees true 3D environment, not triple front

## Files Modified

- `app.py`:
  - NEW: `_seek_analyze_front_view()` (line 7893)
  - NEW: `_seek_analyze_side_view()` (line 7964)
  - NEW: `_seek_loop()` (line 8019) - complete rewrite
  - OLD: `_seek_unified_llm_analysis()` - STILL EXISTS but not used by new loop
  - NEW: `_seek_wait_pan_arrived()` (line 5723) - wait before snap, not after

## Verification

To verify new code is active:
1. Check logs for: `Step N: LLM front analysis (3 questions)`
2. Should appear EVERY step, not every 10
3. Check logs for: `Pan to 0° complete: settled=True reason=...`
4. Should show settle times 2-3s for rear angles, not 0.5s
5. Check logs for: `Captured left view: cmd=-135° hw=...`
6. If hw=-0.3° but reason=time_elapsed_hw_stuck_at_zero, settle fix working
