// Menu.cpp — implementations for Menu.h. See lib/menu.py for the model.

#include "Menu.h"
#include <cstdio>
#include <cstring>

namespace rls {

namespace {
constexpr int kPixelYShift = 20;
constexpr int kLineHeight  = 10;
constexpr int kSpacer      = 2;
}  // namespace

// ============================================================
//  Helpers
// ============================================================
void CompactName(const char* in, char* out, int out_cap) {
  int w = 0;
  for (const char* p = in; *p && w + 1 < out_cap; ++p) {
    char c = *p;
    bool vowel = (c=='a'||c=='e'||c=='i'||c=='o'||c=='u'||
                  c=='A'||c=='E'||c=='I'||c=='O'||c=='U');
    if (!vowel) out[w++] = c;
  }
  out[w] = 0;
}

// ============================================================
//  SingleSelectVerticalScrollMenu
// ============================================================
SingleSelectVerticalScrollMenu::SingleSelectVerticalScrollMenu(
    const char* name, const char* const* items, int items_count, int selected_index)
    : Submenu(name, Kind::kSingleSelect),
      items_(items), items_count_(items_count), selected_index_(selected_index) {}

void SingleSelectVerticalScrollMenu::Repr(char* out, int out_cap) const {
  const char* sel = items_[selected_index_];
  char compact[24];
  const char* shown = sel;
  if (static_cast<int>(std::strlen(sel)) >= 9) {
    CompactName(sel, compact, sizeof(compact));
    shown = compact;
  }
  std::snprintf(out, out_cap, "%s:%s", name_, shown);
}

void SingleSelectVerticalScrollMenu::Start(MainMenu& parent) {
  highlighted_index_ = 0;
  menu_start_index_  = 0;
  parent.set_rotary_bounds(0, 0, items_count_ - 1, 1);
}

void SingleSelectVerticalScrollMenu::Scroll(int idx) {
  if (idx > menu_start_index_ + (total_lines_ - 1)) menu_start_index_++;
  if (idx < menu_start_index_)                      menu_start_index_--;
}

void SingleSelectVerticalScrollMenu::OnRotate(MainMenu& parent, int /*delta*/) {
  const int v = parent.rotary_value();
  Scroll(v);
  highlighted_index_ = v;
}

bool SingleSelectVerticalScrollMenu::Commit(MainMenu& parent) {
  const int prev = selected_index_;
  selected_index_ = parent.rotary_value();
  return selected_index_ != prev;
}

void SingleSelectVerticalScrollMenu::Draw(FakeOled& oled) const {
  oled.Clear();
  char header[24];
  const char* sel = items_[selected_index_];
  char compact[24];
  const char* shown = sel;
  if (static_cast<int>(std::strlen(sel)) > 9) {
    CompactName(sel, compact, sizeof(compact));
    shown = compact;
  }
  std::snprintf(header, sizeof(header), "%s:%s", name_, shown);
  oled.Text(2, 4, header);
  oled.Rect(0, 0, 128, 15);

  const int visible = items_count_ - menu_start_index_;
  const int rows    = (visible < total_lines_) ? visible : total_lines_;
  for (int i = 0; i < rows; ++i) {
    const int item_index = menu_start_index_ + i;
    const int y = (i * (kLineHeight + kSpacer)) + kPixelYShift;
    const bool highlighted = (item_index == highlighted_index_);
    const bool is_selected = (selected_index_ == item_index);

    char line[24];
    std::snprintf(line, sizeof(line), "%s%s",
                  is_selected ? "*" : "",
                  items_[item_index]);

    if (highlighted) {
      oled.FillRect(0, y - 1, 128, kLineHeight, true);
      oled.Text(0, y, line, false);   // black on white
    } else {
      oled.Text(0, y, line, true);
    }
  }
}

// ============================================================
//  NumericalValueRangeMenu
// ============================================================
NumericalValueRangeMenu::NumericalValueRangeMenu(
    const char* name, int selected, int min_val, int max_val, int increment)
    : Submenu(name, Kind::kNumeric),
      selected_(selected), new_value_(selected),
      min_val_(min_val), max_val_(max_val), increment_(increment) {}

void NumericalValueRangeMenu::Repr(char* out, int out_cap) const {
  std::snprintf(out, out_cap, "%s:%d", name_, selected_);
}

void NumericalValueRangeMenu::Start(MainMenu& parent) {
  new_value_ = selected_;
  parent.set_rotary_bounds(selected_, min_val_, max_val_, increment_);
}

void NumericalValueRangeMenu::OnRotate(MainMenu& parent, int /*delta*/) {
  new_value_ = parent.rotary_value();
}

bool NumericalValueRangeMenu::Commit(MainMenu& parent) {
  const int prev = selected_;
  selected_ = parent.rotary_value();
  return selected_ != prev;
}

void NumericalValueRangeMenu::Draw(FakeOled& oled) const {
  // Horizontal-bar editor — replaces the OLD/NEW two-line text from main.py.
  // Layout (128x64 OLED):
  //   y=0..14   : title bar (name)
  //   y=18      : big current value, centred
  //   y=36..43  : horizontal bar (fill = position of new_value_ in min..max)
  //   y=48      : min / max labels
  //   y=56      : "OLD n" small text on the left
  oled.Clear();
  oled.Text(2, 4, name_);
  oled.Rect(0, 0, 128, 15);

  // Big value, centred (each glyph = 6 px wide in our FakeOled font)
  char val_str[12];
  std::snprintf(val_str, sizeof(val_str), "%d", new_value_);
  const int val_w = static_cast<int>(std::strlen(val_str)) * 6;
  const int val_x = (128 - val_w) / 2;
  oled.Text(val_x, 18, val_str);

  // Bar (4 px margin each side, 8 px tall)
  constexpr int kBarX = 4, kBarY = 36, kBarW = 120, kBarH = 8;
  oled.Rect(kBarX, kBarY, kBarW, kBarH);
  const int span = (max_val_ - min_val_);
  const int fill = (span > 0)
      ? ((new_value_ - min_val_) * (kBarW - 2)) / span
      : 0;
  if (fill > 0) {
    oled.FillRect(kBarX + 1, kBarY + 1, fill, kBarH - 2);
  }

  // Min and max labels under the bar
  char min_lbl[8], max_lbl[8];
  std::snprintf(min_lbl, sizeof(min_lbl), "%d", min_val_);
  std::snprintf(max_lbl, sizeof(max_lbl), "%d", max_val_);
  oled.Text(kBarX, 48, min_lbl);
  const int max_w = static_cast<int>(std::strlen(max_lbl)) * 6;
  oled.Text(kBarX + kBarW - max_w, 48, max_lbl);

  // "OLD n" reminder
  char old_lbl[16];
  std::snprintf(old_lbl, sizeof(old_lbl), "OLD %d", selected_);
  oled.Text(0, 56, old_lbl);
}

// ============================================================
//  ToggleMenu
// ============================================================
ToggleMenu::ToggleMenu(const char* name, bool value)
    : Submenu(name, Kind::kToggle), value_(value) {}

void ToggleMenu::Repr(char* out, int out_cap) const {
  std::snprintf(out, out_cap, "%s:%s", name_, value_ ? "ON" : "OFF");
}

void ToggleMenu::Start(MainMenu& /*parent*/)               {}
void ToggleMenu::OnRotate(MainMenu& /*parent*/, int)       {}
bool ToggleMenu::Commit(MainMenu& /*parent*/)              { return true; }
void ToggleMenu::Draw(FakeOled& /*oled*/) const            {}

// ============================================================
//  MainMenu
// ============================================================
void MainMenu::SetSubmenus(Submenu* const* submenus, int count) {
  submenus_       = submenus;
  submenus_count_ = count;
  // Initialise rotary for the main list bounds.
  set_rotary_bounds(0, 0, count - 1, 1);
  highlighted_index_  = 0;
  menu_start_index_   = 0;
  submenu_editing_    = false;
  current_menu_index_ = -1;
  current_submenu_    = nullptr;
}

void MainMenu::ApplyRotaryDelta(int delta) {
  int v = rotary_value_ + delta * rotary_incr_;
  if (v < rotary_min_) v = rotary_min_;
  if (v > rotary_max_) v = rotary_max_;
  rotary_value_ = v;
}

void MainMenu::OnRotate(int delta) {
  ApplyRotaryDelta(delta);
  if (submenu_editing_ && current_submenu_) {
    current_submenu_->OnRotate(*this, delta);
  } else {
    ScrollMainMenu(rotary_value_);
    highlighted_index_ = rotary_value_;
  }
}

void MainMenu::ScrollMainMenu(int index) {
  if (index > menu_start_index_ + (total_lines_ - 1)) menu_start_index_++;
  if (index < menu_start_index_)                      menu_start_index_--;
}

void MainMenu::OnPress() {
  if (!submenus_ || submenus_count_ == 0) return;

  if (current_menu_index_ < 0) {
    // Pressed in main menu — latch selection.
    Submenu* picked = submenus_[highlighted_index_];
    current_submenu_ = picked;
    if (picked->IsToggleStyle()) {
      static_cast<ToggleMenu*>(picked)->Toggle();
      // Stay in main menu, fire commit so host syncs params.
      if (commit_cb_) commit_cb_(commit_user_);
    } else {
      submenu_editing_    = true;
      current_menu_index_ = highlighted_index_;
      picked->Start(*this);
    }
  } else {
    // Pressed inside a submenu — commit and return to main.
    if (current_submenu_) current_submenu_->Commit(*this);
    submenu_editing_    = false;
    current_menu_index_ = -1;
    // Restore main-list rotary bounds at the previously highlighted row.
    set_rotary_bounds(highlighted_index_, 0, submenus_count_ - 1, 1);
    current_submenu_    = nullptr;
    if (commit_cb_) commit_cb_(commit_user_);
  }
}

void MainMenu::DrawMainList(FakeOled& oled) const {
  oled.Clear();
  oled.Text(2, 4, "MAIN MENU");
  oled.Rect(0, 0, 128, 15);

  const int visible = submenus_count_ - menu_start_index_;
  const int rows    = (visible < total_lines_) ? visible : total_lines_;
  for (int i = 0; i < rows; ++i) {
    const int item_index = menu_start_index_ + i;
    const int y = (i * (kLineHeight + kSpacer)) + kPixelYShift;
    char line[24];
    submenus_[item_index]->Repr(line, sizeof(line));

    if (item_index == highlighted_index_) {
      oled.FillRect(0, y - 1, 128, kLineHeight, true);
      oled.Text(0, y, line, false);
    } else {
      oled.Text(0, y, line, true);
    }
  }
}

void MainMenu::Draw(FakeOled& oled) const {
  if (submenu_editing_ && current_submenu_) {
    current_submenu_->Draw(oled);
  } else {
    DrawMainList(oled);
  }
}

}  // namespace rls
