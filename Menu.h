// Menu.h
// View-stack-based menu system for the OLED + encoder UI.
//
// Replaces the v0.2 design (MainMenu owning a flat list of Submenu*
// with a 3-boolean state machine) with:
//
//   View       — anything that draws to the OLED and consumes encoder
//                events. Long-press cancels by popping itself off the
//                view stack without committing.
//   ViewStack  — the navigation history. Push to enter, pop to leave.
//                Hierarchical navigation comes for free once we go
//                polyphonic (a top-level "Voice 1 → Voice 2 → …" view
//                sits above the per-voice list).
//   MenuItem   — a row in a MenuListView. Pressing the row either
//                pushes an editor view (numeric, single-select), or
//                fires inline (toggle). Different from a View because
//                an item only ever exists *inside* a list — it doesn't
//                draw a full screen on its own.
//
// This is the Phase-A architecture. Behaviour parity with v0.2 is the
// goal of the first commit; feature work (long-press cancel, toggle
// flash, scale-name truncation, etc.) lands on this skeleton in Phase B.

#pragma once
#include <cstdint>
#include "FakeOled.h"

namespace seq {

class ViewStack;
class View;
class MenuItem;
class MenuListView;

// ============================================================
//  View
// ============================================================
class View {
 public:
  virtual ~View() = default;
  virtual void Draw(FakeOled& oled) const = 0;
  // Encoder events. Defaults are no-ops so subclasses only override what
  // they care about.
  virtual void OnRotate(int /*delta*/) {}
  virtual void OnPress()               {}
  virtual void OnLongPress();   // default: pop self if poppable
  void set_stack(ViewStack* s) { stack_ = s; }
  ViewStack* stack() const     { return stack_; }
 protected:
  ViewStack* stack_ = nullptr;
};

// ============================================================
//  ViewStack
// ============================================================
class ViewStack {
 public:
  static constexpr int kMaxDepth = 8;

  void Push(View* v);
  void Pop();
  View* top() const { return depth_ > 0 ? stack_[depth_ - 1] : nullptr; }
  int   depth() const { return depth_; }
  bool  can_pop() const { return depth_ > 1; }

  void Draw(FakeOled& oled) const { if (View* v = top()) v->Draw(oled); }
  void OnRotate(int delta)        { if (View* v = top()) v->OnRotate(delta); }
  void OnPress()                  { if (View* v = top()) v->OnPress(); }
  void OnLongPress()              { if (View* v = top()) v->OnLongPress(); }

 private:
  View* stack_[kMaxDepth] = {nullptr};
  int   depth_            = 0;
};

// ============================================================
//  MenuItem — a row in a MenuListView
// ============================================================
class MenuItem {
 public:
  virtual ~MenuItem() = default;
  virtual const char* name() const = 0;
  // Writes a line like "CVProb:30" or "Scale:major" into out.
  virtual void Repr(char* out, int cap) const = 0;
  // Pressed while highlighted. Returns a View* to push (the editor),
  // or nullptr to stay in the list. Toggle items mutate inline.
  // `list` is the parent MenuListView (so editors can notify on commit).
  virtual View* OnPressInList(ViewStack& stack, MenuListView& list) = 0;
};

// ============================================================
//  MenuListView — the root navigation list (and any nested list)
// ============================================================
class MenuListView : public View {
 public:
  void SetTitle(const char* t) { title_ = t; }
  void SetItems(MenuItem* const* items, int count);

  typedef void (*CommitCallback)(void* user);
  void SetCommitCallback(CommitCallback cb, void* user) {
    commit_cb_ = cb;
    commit_user_ = user;
  }
  void NotifyCommit() { if (commit_cb_) commit_cb_(commit_user_); }

  void Draw(FakeOled& oled) const override;
  void OnRotate(int delta) override;
  void OnPress() override;

  int highlighted_index() const { return highlighted_; }
  int item_count()         const { return count_; }
  MenuItem* item(int i)    const { return items_ ? items_[i] : nullptr; }

  // Direct jump to a specific item index. Used by the sim's number-key
  // shortcut (1..9). Adjusts scroll window so the target is visible.
  void JumpTo(int index);

  // Audit 3c: brief inverse-flash on a row after an inline-action press
  // (toggle or action item). MenuListView::OnPress sets these automatically
  // when an item's OnPressInList returns nullptr.
  static constexpr int kFlashFrames = 8;   // ~135 ms at 60Hz

 private:
  const char*       title_           = "Menu";
  MenuItem* const*  items_           = nullptr;
  int               count_           = 0;
  int               highlighted_     = 0;
  int               menu_start_idx_  = 0;
  int               total_lines_     = 4;
  CommitCallback    commit_cb_       = nullptr;
  void*             commit_user_     = nullptr;
  // Flash state — set by OnPress, decremented in Draw.
  int               flash_index_                = -1;
  mutable int       flash_frames_remaining_     = 0;
};

// ============================================================
//  Concrete MenuItems
// ============================================================
class ToggleItem : public MenuItem {
 public:
  ToggleItem(const char* name, bool initial) : name_(name), value_(initial) {}
  const char* name() const override { return name_; }
  void Repr(char* out, int cap) const override;
  View* OnPressInList(ViewStack& stack, MenuListView& list) override;
  bool value() const { return value_; }
  void set_value(bool v) { value_ = v; }
 private:
  const char* name_;
  bool        value_;
};

class NumericalItem : public MenuItem {
 public:
  NumericalItem(const char* name, int initial,
                int min_v, int max_v, int step)
      : name_(name), value_(initial),
        min_(min_v), max_(max_v), step_(step) {}
  const char* name() const override { return name_; }
  void Repr(char* out, int cap) const override;
  View* OnPressInList(ViewStack& stack, MenuListView& list) override;

  int  value() const { return value_; }
  void set_value(int v) { value_ = v; }
  int  min_val() const { return min_; }
  int  max_val() const { return max_; }
  int  step()    const { return step_; }

 private:
  const char* name_;
  int value_, min_, max_, step_;
};

// One-shot action — fires a callback immediately on press, no editor pushed.
// Used for "Clear CV" / "Clear Trig" etc. (audit 2a) where the user wants
// a single irreversible side-effect, not an ongoing mode toggle.
class ActionItem : public MenuItem {
 public:
  typedef void (*Action)(void* user);
  ActionItem(const char* name, Action action, void* user)
      : name_(name), action_(action), user_(user) {}
  const char* name() const override { return name_; }
  void  Repr(char* out, int cap) const override;
  View* OnPressInList(ViewStack& stack, MenuListView& list) override;
 private:
  const char* name_;
  Action      action_;
  void*       user_;
};

class SingleSelectItem : public MenuItem {
 public:
  SingleSelectItem(const char* name,
                   const char* const* options, int option_count,
                   int initial_index)
      : name_(name), options_(options), option_count_(option_count),
        selected_(initial_index) {}
  const char* name() const override { return name_; }
  void Repr(char* out, int cap) const override;
  View* OnPressInList(ViewStack& stack, MenuListView& list) override;

  int  selected_index() const { return selected_; }
  void set_selected_index(int i) { selected_ = i; }
  const char* selected_label() const { return options_[selected_]; }
  const char* const* options() const { return options_; }
  int  option_count() const { return option_count_; }

 private:
  const char*        name_;
  const char* const* options_;
  int                option_count_;
  int                selected_;
};

// ============================================================
//  Editor views — pushed by NumericalItem / SingleSelectItem
// ============================================================
class NumericalEditView : public View {
 public:
  NumericalEditView(NumericalItem* item, MenuListView* parent)
      : item_(item), parent_(parent), draft_(item->value()) {}
  void Draw(FakeOled& oled) const override;
  void OnRotate(int delta) override;
  void OnPress() override;       // commit draft, pop
  // OnLongPress inherits default (pop without committing) — audit 3b.
 private:
  NumericalItem* item_;
  MenuListView*  parent_;
  int            draft_;
};

class SingleSelectEditView : public View {
 public:
  SingleSelectEditView(SingleSelectItem* item, MenuListView* parent)
      : item_(item), parent_(parent),
        draft_index_(item->selected_index()),
        scroll_start_(0) {}
  void Draw(FakeOled& oled) const override;
  void OnRotate(int delta) override;
  void OnPress() override;
 private:
  SingleSelectItem* item_;
  MenuListView*     parent_;
  int               draft_index_;
  int               scroll_start_;
};

}  // namespace seq
