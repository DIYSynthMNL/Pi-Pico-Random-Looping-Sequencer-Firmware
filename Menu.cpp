// Menu.cpp — see Menu.h.

#include "Menu.h"
#include <chrono>
#include <cstdio>
#include <cstring>

namespace seq {

namespace {
// Local time helper — kept private to Menu.cpp so the menu library
// doesn't take a dep on playground globals.
uint32_t MenuNowMs() {
  return static_cast<uint32_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count());
}
}  // namespace

namespace {
constexpr int kPixelYShift = 20;
constexpr int kLineHeight  = 10;
constexpr int kSpacer      = 2;

// ============================================================
//  Row icons — drawn at the start of each MenuListView row to hint
//  what pressing the row does. Each is a 6x8 bitmap (column-major,
//  bit 0 = top row — same encoding as the FakeOled font).
// ============================================================
struct RowIcon { uint8_t cols[6]; };

// `<` chevron pointing left — back / pop the view stack
constexpr RowIcon kIconBack    = {{0x18, 0x3C, 0x66, 0xC3, 0x00, 0x00}};
// `>` chevron pointing right — descend into a submenu
constexpr RowIcon kIconForward = {{0xC3, 0x66, 0x3C, 0x18, 0x00, 0x00}};
// outlined small square — boolean toggle
constexpr RowIcon kIconToggle  = {{0x3C, 0x24, 0x24, 0x24, 0x3C, 0x00}};
// 3 short horizontal lines stacked — slider / numerical editor
constexpr RowIcon kIconSlider  = {{0x2A, 0x2A, 0x2A, 0x2A, 0x00, 0x00}};
// diamond — single-select or grid-select editor
constexpr RowIcon kIconSelect  = {{0x08, 0x14, 0x22, 0x14, 0x08, 0x00}};
// exclamation mark — one-shot action (destructive)
constexpr RowIcon kIconAction  = {{0x00, 0x00, 0xBF, 0x00, 0x00, 0x00}};

static void DrawRowIcon(FakeOled& oled, const RowIcon& icon,
                        int x, int y, bool on) {
  for (int col = 0; col < 6; ++col) {
    const uint8_t bits = icon.cols[col];
    for (int row = 0; row < 8; ++row) {
      if (bits & (1u << row)) oled.Px(x + col, y + row, on);
    }
  }
}

static const RowIcon& IconForKind(MenuItemKind k) {
  switch (k) {
    case MenuItemKind::Back:         return kIconBack;
    case MenuItemKind::Submenu:      return kIconForward;
    case MenuItemKind::Toggle:       return kIconToggle;
    case MenuItemKind::Numerical:    return kIconSlider;
    case MenuItemKind::SingleSelect: return kIconSelect;
    case MenuItemKind::GridSelect:   return kIconSelect;
    case MenuItemKind::Action:       return kIconAction;
  }
  return kIconForward;
}
}  // namespace

// ============================================================
//  View defaults
// ============================================================
void View::OnLongPress() {
  // Default: pop self off the stack if there's somewhere to return to.
  // Editors override nothing; root views (depth 1) ignore.
  if (stack_ && stack_->can_pop()) stack_->Pop();
}

// ============================================================
//  ViewStack
// ============================================================
void ViewStack::Push(View* v) {
  if (!v || depth_ >= kMaxDepth) return;
  // Start a Pushing transition. Any in-flight transition collapses
  // immediately — the new push wins.
  outgoing_view_         = top();
  transition_            = (outgoing_view_) ? Transition::Pushing : Transition::None;
  transition_started_ms_ = MenuNowMs();
  v->set_stack(this);
  stack_[depth_++] = v;
}

void ViewStack::Pop() {
  if (depth_ <= 0) return;
  // Start a Popping transition. outgoing_view_ is the view leaving the
  // stack — held as an observer pointer for the kTransitionMs window
  // (it may be heap-owned by the caller; the leak is pre-existing and
  // tracked separately).
  outgoing_view_         = top();
  transition_            = (outgoing_view_) ? Transition::Popping : Transition::None;
  transition_started_ms_ = MenuNowMs();
  --depth_;
  stack_[depth_] = nullptr;
}

void ViewStack::Draw(FakeOled& oled) {
  View* current = top();

  if (transition_ == Transition::None || outgoing_view_ == nullptr) {
    if (current) current->Draw(oled);
    return;
  }
  const uint32_t age = MenuNowMs() - transition_started_ms_;
  if (age >= kTransitionMs) {
    transition_    = Transition::None;
    outgoing_view_ = nullptr;
    if (current) current->Draw(oled);
    return;
  }

  // Mid-transition — render both views into temp buffers and composite
  // with a horizontal slide. Push: outgoing → left, incoming ← right.
  // Pop: opposite.
  FakeOled out_buf;
  FakeOled in_buf;
  outgoing_view_->Draw(out_buf);
  if (current) current->Draw(in_buf);

  const uint32_t kW = 128;
  const uint32_t slide = (kW * age) / kTransitionMs;
  int out_offset, in_offset;
  if (transition_ == Transition::Pushing) {
    out_offset = -static_cast<int>(slide);
    in_offset  =  static_cast<int>(kW - slide);
  } else {  // Popping
    out_offset =  static_cast<int>(slide);
    in_offset  = -static_cast<int>(kW - slide);
  }

  oled.Clear();
  for (int y = 0; y < 64; ++y) {
    // Compose outgoing
    for (int x = 0; x < 128; ++x) {
      const int sx = x - out_offset;
      if (sx >= 0 && sx < 128 && out_buf.buf[y * 128 + sx]) {
        oled.buf[y * 128 + x] = 255;
      }
    }
    // Compose incoming on top
    for (int x = 0; x < 128; ++x) {
      const int sx = x - in_offset;
      if (sx >= 0 && sx < 128 && in_buf.buf[y * 128 + sx]) {
        oled.buf[y * 128 + x] = 255;
      }
    }
  }
}

// ============================================================
//  MenuListView
// ============================================================
void MenuListView::SetItems(MenuItem* const* items, int count) {
  items_          = items;
  count_          = count;
  highlighted_    = 0;
  menu_start_idx_ = 0;
}

void MenuListView::OnRotate(int delta) {
  highlighted_ += delta;
  if (highlighted_ < 0)              highlighted_ = 0;
  if (highlighted_ >= count_)        highlighted_ = count_ - 1;
  if (highlighted_ > menu_start_idx_ + (total_lines_ - 1)) ++menu_start_idx_;
  if (highlighted_ < menu_start_idx_)                      --menu_start_idx_;
  if (menu_start_idx_ < 0) menu_start_idx_ = 0;
}

void MenuListView::JumpTo(int index) {
  if (!items_ || index < 0 || index >= count_) return;
  highlighted_ = index;
  if (highlighted_ > menu_start_idx_ + (total_lines_ - 1))
    menu_start_idx_ = highlighted_ - (total_lines_ - 1);
  if (highlighted_ < menu_start_idx_)
    menu_start_idx_ = highlighted_;
  if (menu_start_idx_ < 0) menu_start_idx_ = 0;
}

void MenuListView::OnPress() {
  if (!items_ || highlighted_ < 0 || highlighted_ >= count_) return;
  View* editor = items_[highlighted_]->OnPressInList(*stack(), *this);
  if (editor) {
    stack()->Push(editor);
  } else {
    // Inline action (toggle or action item) — flash the row to acknowledge
    // the press, then fire the commit so the host syncs params.
    flash_index_            = highlighted_;
    flash_frames_remaining_ = kFlashFrames;
    NotifyCommit();
  }
}

void MenuListView::Draw(FakeOled& oled) const {
  oled.Clear();
  oled.Text(2, 4, title_);
  oled.Rect(0, 0, 128, 15);

  const int visible = count_ - menu_start_idx_;
  const int rows    = (visible < total_lines_) ? visible : total_lines_;
  for (int i = 0; i < rows; ++i) {
    const int item_index = menu_start_idx_ + i;
    const int y = (i * (kLineHeight + kSpacer)) + kPixelYShift;
    char line[24];
    items_[item_index]->Repr(line, sizeof(line));

    const bool is_highlighted = (item_index == highlighted_);
    const bool is_flashing    = (item_index == flash_index_ &&
                                 flash_frames_remaining_ > 0);
    const bool invert         = is_highlighted && !is_flashing;

    if (invert) oled.FillRect(0, y - 1, 128, kLineHeight, true);

    // Type-hint icon at the start of each row, then the label shifted
    // 8 px right. Icon colour matches the row's text colour.
    const bool on = !invert;
    DrawRowIcon(oled, IconForKind(items_[item_index]->kind()), 0, y, on);
    oled.Text(8, y, line, on);
  }
  if (flash_frames_remaining_ > 0) --flash_frames_remaining_;
}

// ============================================================
//  BackItem
// ============================================================
void BackItem::Repr(char* out, int cap) const {
  std::snprintf(out, cap, "Back");
}

View* BackItem::OnPressInList(ViewStack& stack, MenuListView& /*list*/) {
  if (stack.can_pop()) stack.Pop();
  return nullptr;   // stay in caller's notify-commit path (harmless)
}

// ============================================================
//  ToggleItem
// ============================================================
void ToggleItem::Repr(char* out, int cap) const {
  std::snprintf(out, cap, "%s:%s", name_, value_ ? "ON" : "OFF");
}

View* ToggleItem::OnPressInList(ViewStack& /*stack*/,
                                MenuListView& /*list*/) {
  value_ = !value_;
  return nullptr;   // stay in list; MenuListView fires commit callback
}

// ============================================================
//  NumericalItem
// ============================================================
void NumericalItem::Repr(char* out, int cap) const {
  std::snprintf(out, cap, "%s:%d", name_, value_);
}

// ============================================================
//  SubmenuItem
// ============================================================
void SubmenuItem::Repr(char* out, int cap) const {
  // Just the name — the row icon (a `>` chevron) already signals "press
  // to descend", so we don't suffix the name with a redundant arrow.
  std::snprintf(out, cap, "%s", name_);
}

View* SubmenuItem::OnPressInList(ViewStack& /*stack*/,
                                 MenuListView& /*list*/) {
  return child_;     // ViewStack::Push handles ownership / depth
}

// ============================================================
//  ActionItem
// ============================================================
void ActionItem::Repr(char* out, int cap) const {
  // No "Name:value" — actions don't carry persistent state. Just the name.
  std::snprintf(out, cap, "%s", name_);
}

View* ActionItem::OnPressInList(ViewStack& /*stack*/,
                                MenuListView& list) {
  if (confirm_) {
    const char* prompt = confirm_prompt_ ? confirm_prompt_ : name_;
    return new ConfirmView(prompt, action_, user_, &list);
  }
  if (action_) action_(user_);
  return nullptr;
}

// ============================================================
//  ConfirmView
// ============================================================
void ConfirmView::OnRotate(int delta) {
  // Two options; toggle between them.
  highlighted_ = (highlighted_ + delta + 2) % 2;
}

void ConfirmView::OnPress() {
  const bool said_yes = (highlighted_ == 1);
  if (stack_) stack_->Pop();
  if (said_yes && on_yes_) on_yes_(user_);
  if (said_yes && parent_) parent_->NotifyCommit();
}

void ConfirmView::Draw(FakeOled& oled) const {
  oled.Clear();
  oled.Text(2, 4, "Confirm");
  oled.Rect(0, 0, 128, 15);

  // Prompt centred just below the title bar
  const int prompt_w = static_cast<int>(std::strlen(prompt_)) * 6;
  const int prompt_x = (128 - prompt_w) / 2;
  oled.Text(prompt_x, 22, prompt_);

  // No / Yes buttons centred horizontally below the prompt
  constexpr int kBtnW  = 50;
  constexpr int kBtnH  = 18;
  constexpr int kBtnY  = 40;
  constexpr int kGap   = 8;
  const int left_x  = (128 - 2 * kBtnW - kGap) / 2;
  const int right_x = left_x + kBtnW + kGap;

  // No button — left
  if (highlighted_ == 0) oled.FillRect(left_x, kBtnY, kBtnW, kBtnH, true);
  else                   oled.Rect    (left_x, kBtnY, kBtnW, kBtnH);
  {
    const int tw = 12;  // "NO" = 2 chars × 6 px
    oled.Text(left_x + (kBtnW - tw) / 2, kBtnY + (kBtnH - 8) / 2,
              "NO", highlighted_ != 0);
  }

  // Yes button — right
  if (highlighted_ == 1) oled.FillRect(right_x, kBtnY, kBtnW, kBtnH, true);
  else                   oled.Rect    (right_x, kBtnY, kBtnW, kBtnH);
  {
    const int tw = 18;  // "YES" = 3 chars × 6 px
    oled.Text(right_x + (kBtnW - tw) / 2, kBtnY + (kBtnH - 8) / 2,
              "YES", highlighted_ != 1);
  }
}

View* NumericalItem::OnPressInList(ViewStack& /*stack*/,
                                   MenuListView& list) {
  // Hand off ownership to the stack — ViewStack does not delete views
  // (caller pattern in playground.cpp manages lifetimes), so allocate.
  return new NumericalEditView(this, &list);
}

// ============================================================
//  SingleSelectItem
// ============================================================
void SingleSelectItem::Repr(char* out, int cap) const {
  // 6x8 font, 128 px wide OLED → 21 glyph columns. Cap to 20 chars to
  // keep a 1-glyph right margin. Truncate the *value* with "..." rather
  // than chopping characters mid-name (the old vowel-strip from main.py
  // produced unreadable results like "ctntnc c-c#"). Audit 3a.
  constexpr int kMaxChars = 20;
  const int prefix_n = std::snprintf(out, cap, "%s:", name_);
  if (prefix_n < 0 || prefix_n >= cap) return;

  const char* value = options_[selected_];
  const int   value_len = static_cast<int>(std::strlen(value));
  const int   remaining = kMaxChars - prefix_n;

  if (value_len <= remaining) {
    std::snprintf(out + prefix_n, cap - prefix_n, "%s", value);
    return;
  }
  // Truncate with "..." (three ASCII dots — visible in our 6x8 font).
  if (remaining > 3 && remaining < cap - prefix_n) {
    const int copy_n = remaining - 3;
    std::memcpy(out + prefix_n, value, copy_n);
    out[prefix_n + copy_n + 0] = '.';
    out[prefix_n + copy_n + 1] = '.';
    out[prefix_n + copy_n + 2] = '.';
    out[prefix_n + copy_n + 3] = 0;
  } else {
    // Vanishingly small budget — just hard-truncate.
    const int n = (remaining < cap - prefix_n) ? remaining : (cap - prefix_n - 1);
    std::memcpy(out + prefix_n, value, n);
    out[prefix_n + n] = 0;
  }
}

View* SingleSelectItem::OnPressInList(ViewStack& /*stack*/,
                                      MenuListView& list) {
  return new SingleSelectEditView(this, &list);
}

// ============================================================
//  GridSelectItem
// ============================================================
void GridSelectItem::Repr(char* out, int cap) const {
  // Same format as SingleSelectItem — "Name:value" on the main list row.
  std::snprintf(out, cap, "%s:%s", name_, options_[selected_]);
}

View* GridSelectItem::OnPressInList(ViewStack& /*stack*/,
                                    MenuListView& list) {
  return new GridSelectEditView(this, &list);
}

// ============================================================
//  GridSelectEditView
// ============================================================
void GridSelectEditView::OnRotate(int delta) {
  draft_index_ += delta;
  if (draft_index_ < 0)                                draft_index_ = 0;
  if (draft_index_ >= item_->option_count())           draft_index_ = item_->option_count() - 1;
}

void GridSelectEditView::OnPress() {
  item_->set_selected_index(draft_index_);
  if (stack_) stack_->Pop();
  if (parent_) parent_->NotifyCommit();
}

void GridSelectEditView::Draw(FakeOled& oled) const {
  oled.Clear();
  oled.Text(2, 4, item_->name());
  oled.Rect(0, 0, 128, 15);

  const int cols  = (item_->cols() < 1) ? 1 : item_->cols();
  const int count = item_->option_count();
  const int rows  = (count + cols - 1) / cols;

  // Body area below the title bar.
  constexpr int kBodyTop = 17;
  constexpr int kBodyBot = 63;
  const int     body_h   = kBodyBot - kBodyTop;
  const int     cell_w   = 128 / cols;
  const int     cell_h   = (rows > 0) ? (body_h / rows) : body_h;

  const int prev_selected = item_->selected_index();

  for (int i = 0; i < count; ++i) {
    const int col   = i % cols;
    const int row   = i / cols;
    const int x     = col * cell_w;
    const int y     = kBodyTop + row * cell_h;
    const int w     = (col == cols - 1) ? (128 - x) : cell_w;
    const int h     = cell_h;
    const bool draft = (i == draft_index_);
    const bool was  = (i == prev_selected);

    if (draft) {
      oled.FillRect(x, y, w, h, true);
    } else {
      oled.Rect(x, y, w, h);
    }

    // Centre the label inside the cell.
    const char* lbl = item_->options()[i];
    const int   len = static_cast<int>(std::strlen(lbl));
    const int   tx  = x + (w - len * 6) / 2;
    const int   ty  = y + (h - 8) / 2;
    oled.Text(tx, ty, lbl, !draft);

    // Mark the previously-committed option with a small dot in the
    // top-left corner so the user remembers what was set before.
    if (was && !draft) {
      oled.Px(x + 2, y + 2, true);
    }
    if (was && draft) {
      // On inverted background, draw the marker as an off pixel so it
      // still reads.
      oled.Px(x + 2, y + 2, false);
    }
  }
}

// ============================================================
//  NumericalEditView
// ============================================================
void NumericalEditView::OnRotate(int delta) {
  draft_ += delta * item_->step();
  if (draft_ < item_->min_val()) draft_ = item_->min_val();
  if (draft_ > item_->max_val()) draft_ = item_->max_val();
}

void NumericalEditView::OnPress() {
  item_->set_value(draft_);
  if (stack_) stack_->Pop();
  if (parent_) parent_->NotifyCommit();
}

void NumericalEditView::Draw(FakeOled& oled) const {
  // Big value + horizontal bar (carried from v0.2's redesign).
  oled.Clear();
  oled.Text(2, 4, item_->name());
  oled.Rect(0, 0, 128, 15);

  char val_str[12];
  std::snprintf(val_str, sizeof(val_str), "%d", draft_);
  const int val_w = static_cast<int>(std::strlen(val_str)) * 6;
  const int val_x = (128 - val_w) / 2;
  oled.Text(val_x, 18, val_str);
  // Chevrons flanking the value — hint that encoder rotation changes
  // it. Reuse the existing row icons.
  DrawRowIcon(oled, kIconBack,    val_x - 10,        18, true);
  DrawRowIcon(oled, kIconForward, val_x + val_w + 4, 18, true);

  constexpr int kBarX = 4, kBarY = 36, kBarW = 120, kBarH = 8;
  oled.Rect(kBarX, kBarY, kBarW, kBarH);
  const int span = (item_->max_val() - item_->min_val());
  const int fill = (span > 0)
      ? ((draft_ - item_->min_val()) * (kBarW - 2)) / span
      : 0;
  if (fill > 0) {
    oled.FillRect(kBarX + 1, kBarY + 1, fill, kBarH - 2);
  }

  char min_lbl[8], max_lbl[8];
  std::snprintf(min_lbl, sizeof(min_lbl), "%d", item_->min_val());
  std::snprintf(max_lbl, sizeof(max_lbl), "%d", item_->max_val());
  oled.Text(kBarX, 48, min_lbl);
  const int max_w = static_cast<int>(std::strlen(max_lbl)) * 6;
  oled.Text(kBarX + kBarW - max_w, 48, max_lbl);

  char old_lbl[16];
  std::snprintf(old_lbl, sizeof(old_lbl), "OLD %d", item_->value());
  oled.Text(0, 56, old_lbl);
}

// ============================================================
//  SingleSelectEditView
// ============================================================
void SingleSelectEditView::OnRotate(int delta) {
  draft_index_ += delta;
  if (draft_index_ < 0)                       draft_index_ = 0;
  if (draft_index_ >= item_->option_count())  draft_index_ = item_->option_count() - 1;
  const int total_lines = 4;
  if (draft_index_ > scroll_start_ + (total_lines - 1)) ++scroll_start_;
  if (draft_index_ < scroll_start_)                     --scroll_start_;
  if (scroll_start_ < 0) scroll_start_ = 0;
}

void SingleSelectEditView::OnPress() {
  item_->set_selected_index(draft_index_);
  if (stack_) stack_->Pop();
  if (parent_) parent_->NotifyCommit();
}

void SingleSelectEditView::Draw(FakeOled& oled) const {
  oled.Clear();

  // Header — show "Name:value (Section)" if the item has section info.
  // The section follows the *draft* (current rotation target), so it
  // updates live as the user scrolls past category boundaries.
  const auto section_fn = item_->section_for_index_fn();
  char header[28];
  if (section_fn) {
    std::snprintf(header, sizeof(header), "%s [%s]",
                  item_->options()[draft_index_],
                  section_fn(draft_index_));
  } else {
    std::snprintf(header, sizeof(header), "%s:%s",
                  item_->name(), item_->options()[draft_index_]);
  }
  oled.Text(2, 4, header);
  oled.Rect(0, 0, 128, 15);

  const int visible = item_->option_count() - scroll_start_;
  const int total_lines = 4;
  const int rows = (visible < total_lines) ? visible : total_lines;
  const int previously_selected = item_->selected_index();
  for (int i = 0; i < rows; ++i) {
    const int item_idx = scroll_start_ + i;
    const int y = (i * (kLineHeight + kSpacer)) + kPixelYShift;

    // Section divider line above this row if section_fn says this index
    // starts a new section (i.e. its section differs from the previous
    // index's section). Pointer equality on the returned strings — see
    // SetSectionForIndex contract.
    if (section_fn && item_idx > 0 &&
        section_fn(item_idx) != section_fn(item_idx - 1)) {
      // Dashed line — every other pixel — to read as a divider not a
      // selection underline.
      for (int x = 0; x < 128; x += 3) oled.Px(x, y - 2, true);
    }

    char line[24];
    std::snprintf(line, sizeof(line), "%s%s",
                  item_idx == previously_selected ? "*" : "",
                  item_->options()[item_idx]);
    if (item_idx == draft_index_) {
      oled.FillRect(0, y - 1, 128, kLineHeight, true);
      oled.Text(0, y, line, false);
    } else {
      oled.Text(0, y, line, true);
    }
  }
}

}  // namespace seq
