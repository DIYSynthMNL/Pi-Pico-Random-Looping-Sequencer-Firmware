// Menu.cpp — see Menu.h.

#include "Menu.h"
#include <cstdio>
#include <cstring>

namespace seq {

namespace {
constexpr int kPixelYShift = 20;
constexpr int kLineHeight  = 10;
constexpr int kSpacer      = 2;
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
  v->set_stack(this);
  stack_[depth_++] = v;
}

void ViewStack::Pop() {
  if (depth_ <= 0) return;
  --depth_;
  stack_[depth_] = nullptr;
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

void MenuListView::OnPress() {
  if (!items_ || highlighted_ < 0 || highlighted_ >= count_) return;
  View* editor = items_[highlighted_]->OnPressInList(*stack(), *this);
  if (editor) {
    stack()->Push(editor);
  } else {
    // Inline action (e.g. toggle) — fire commit now so host syncs params.
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

    if (item_index == highlighted_) {
      oled.FillRect(0, y - 1, 128, kLineHeight, true);
      oled.Text(0, y, line, false);
    } else {
      oled.Text(0, y, line, true);
    }
  }
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
  // Phase A: simple truncate-to-fit (line is 16 chars wide at 6x8).
  // Phase B (audit 3a) replaces this with ellipsis truncation.
  char buf[24];
  std::snprintf(buf, sizeof(buf), "%s:%s", name_, options_[selected_]);
  // Cap to 16 chars so it fits the OLED width without wrapping garbage.
  const int max_len = 20;
  int  len = static_cast<int>(std::strlen(buf));
  if (len > max_len) len = max_len;
  if (len >= cap)    len = cap - 1;
  std::memcpy(out, buf, len);
  out[len] = 0;
}

View* SingleSelectItem::OnPressInList(ViewStack& /*stack*/,
                                      MenuListView& list) {
  return new SingleSelectEditView(this, &list);
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
  char header[24];
  std::snprintf(header, sizeof(header), "%s:%s",
                item_->name(), item_->options()[draft_index_]);
  oled.Text(2, 4, header);
  oled.Rect(0, 0, 128, 15);

  const int visible = item_->option_count() - scroll_start_;
  const int total_lines = 4;
  const int rows = (visible < total_lines) ? visible : total_lines;
  const int previously_selected = item_->selected_index();
  for (int i = 0; i < rows; ++i) {
    const int item_idx = scroll_start_ + i;
    const int y = (i * (kLineHeight + kSpacer)) + kPixelYShift;
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
