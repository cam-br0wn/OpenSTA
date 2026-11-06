// OpenSTA, Static Timing Analyzer
// Copyright (c) 2025, Parallax Software, Inc.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.
//
// The origin of this software must not be misrepresented; you must not
// claim that you wrote the original software.
//
// Altered source versions must be plainly marked as such, and must not be
// misrepresented as being the original software.
//
// This notice may not be removed or altered from any source distribution.

#pragma once

#include <atomic>
#include <cstdint>
#include <utility>
#include <vector>

#include "Vector.hh"
#include "Map.hh"
#include "Iterator.hh"
#include "MinMax.hh"
#include "Transition.hh"
#include "GraphClass.hh"
#include "SearchClass.hh"
#include "Tag.hh"

namespace sta {

class TagGroupBldr;

// Tag -> path index map keyed by Tag::matchCmp equality.
// Lookups probe an open-addressed table with the tag's precomputed match
// hash (Tag::matchCmpHash) and confirm with one matchCmp, instead of walking
// a std::map ordered by TagMatchLess (log2(n) matchCmp calls per lookup for
// the 36 to 150 paths a vertex has with 9 corners). Entries are kept in
// insertion order for iteration; sortedEntries() gives TagMatchLess order,
// which is how path indices are assigned (same as the std::map iteration
// order this replaces, so path indices and tie breaking do not change).
// Erased entries stay in entries_ with tag == nullptr until clear().
class TagMatchMap
{
public:
  struct Entry
  {
    Tag *tag;
    size_t path_index;
  };
  class const_iterator
  {
  public:
    const_iterator(const std::vector<Entry> *entries,
                   size_t index) :
      entries_(entries),
      index_(index)
    {
      skipErased();
    }
    std::pair<Tag*, size_t> operator*() const
    {
      const Entry &entry = (*entries_)[index_];
      return {entry.tag, entry.path_index};
    }
    const_iterator &operator++()
    {
      index_++;
      skipErased();
      return *this;
    }
    bool operator==(const const_iterator &other) const { return index_ == other.index_; }
    bool operator!=(const const_iterator &other) const { return index_ != other.index_; }

  private:
    void skipErased()
    {
      while (index_ < entries_->size() && (*entries_)[index_].tag == nullptr)
        index_++;
    }
    const std::vector<Entry> *entries_;
    size_t index_;
  };

  TagMatchMap(bool match_crpr_clk_pin,
              const StaState *sta,
              size_t capacity = 0);
  // Keeps the table capacity.
  void clear();
  size_t size() const { return size_; }
  bool empty() const { return size_ == 0; }
  bool hasKey(Tag *tag) const;
  void findKey(Tag *tag,
               // Return values.
               size_t &path_index,
               bool &exists) const;
  void findKey(Tag *tag,
               // Return values.
               Tag *&map_tag,
               size_t &path_index,
               bool &exists) const;
  // Insert, or if a matching tag is already present overwrite its path
  // index and keep its tag (std::map::operator[] semantics).
  void insert(Tag *tag,
              size_t path_index);
  // Replace the tag of the entry matching old_tag (same match class).
  void replaceKey(Tag *old_tag,
                  Tag *new_tag);
  void erase(Tag *tag);
  // Insertion order, erased entries included (tag == nullptr).
  const std::vector<Entry> &entries() const { return entries_; }
  // Live entries in TagMatchLess order.
  std::vector<Entry> sortedEntries() const;
  const_iterator begin() const { return const_iterator(&entries_, 0); }
  const_iterator end() const { return const_iterator(&entries_, entries_.size()); }
  bool matchCrprClkPin() const { return match_crpr_clk_pin_; }

private:
  // Probe for tag. Returns the entry index or -1; slot is the table slot
  // holding the entry, or the first free (empty or erased) slot seen.
  int32_t find(Tag *tag,
               size_t &slot) const;
  void rehash(size_t table_size);
  static constexpr int32_t slot_empty = -1;
  static constexpr int32_t slot_erased = -2;

  bool match_crpr_clk_pin_;
  const StaState *sta_;
  std::vector<Entry> entries_;
  std::vector<int32_t> table_;
  size_t size_;   // live entries
  size_t used_;   // table slots that are not empty (live + erased)
};

class TagGroup
{
public:
  TagGroup(TagGroupIndex index,
	   TagMatchMap *path_index_map,
	   bool has_clk_tag,
	   bool has_genclk_src_tag,
	   bool has_filter_tag,
	   bool has_loop_tag,
	   const StaState *sta);
  // For Search::findTagGroup to probe.
  TagGroup(TagGroupBldr *tag_bldr,
	   const StaState *sta);
  ~TagGroup();
  TagGroupIndex index() const { return index_; }
  size_t hash() const { return hash_; }
  void report(const StaState *sta) const;
  void reportArrivalMap(const StaState *sta) const;
  bool hasClkTag() const { return has_clk_tag_; }
  bool hasGenClkSrcTag() const { return has_genclk_src_tag_; }
  bool hasFilterTag() const { return has_filter_tag_; }
  bool hasLoopTag() const { return has_loop_tag_; }
  bool ownPathMap() const { return own_path_map_; }
  size_t pathCount() const { return path_index_map_->size(); }
  void pathIndex(Tag *tag,
                 size_t &path_index,
                 bool &exists) const;
  size_t pathIndex(Tag *tag) const;
  const TagMatchMap *pathIndexMap() const { return path_index_map_; }
  bool hasTag(Tag *tag) const;
  void incrRefCount();
  void decrRefCount();
  int refCount() const { return ref_count_; }

protected:
  static size_t hash(const TagMatchMap *path_index_map,
		     const StaState *sta);

  // tag -> path index
  TagMatchMap *path_index_map_;
  size_t hash_;
  std::atomic<int> ref_count_;
  unsigned int index_:tag_group_index_bits;
  bool has_clk_tag_:1;
  bool has_genclk_src_tag_:1;
  bool has_filter_tag_:1;
  bool has_loop_tag_:1;
  bool own_path_map_:1;
};

class TagGroupHash
{
public:
  size_t operator()(const TagGroup *tag) const;
};

class TagGroupEqual
{
public:
  bool operator()(const TagGroup *group1,
		  const TagGroup *group2) const;
};

class TagGroupBldr
{
public:
  TagGroupBldr(bool match_crpr_clk_pin,
	       const StaState *sta);
  void init(Vertex *vertex);
  bool empty();
  void reportArrivalEntries() const;
  TagGroup *makeTagGroup(TagGroupIndex index,
			 const StaState *sta);
  size_t pathCount() const { return path_index_map_.size(); }
  bool hasClkTag() const { return has_clk_tag_; }
  bool hasGenClkSrcTag() const { return has_genclk_src_tag_; }
  bool hasFilterTag() const { return has_filter_tag_; }
  bool hasLoopTag() const { return has_loop_tag_; }
  bool hasPropagatedClk() const { return has_propagated_clk_; }
  Path *tagMatchPath(Tag *tag);
  void tagMatchPath(Tag *tag,
                    // Return values.
                    Path *&match,
                    size_t &path_index);
  Arrival arrival(size_t path_index) const;
  // prev_path == hull
  void setArrival(Tag *tag,
		  const Arrival &arrival);
  void setMatchPath(Path *match,
                    size_t path_index,
                    Tag *tag,
                    Arrival arrival,
                    Path *prev_path,
                    Edge *prev_edge,
                    TimingArc *prev_arc);
  void insertPath(Tag *tag,
                  Arrival arrival,
                  Path *prev_path,
                  Edge *prev_edge,
                  TimingArc *prev_arc);
  void insertPath(const Path &path);
  TagMatchMap &pathIndexMap() { return path_index_map_; }
  void copyPaths(TagGroup *tag_group,
                 Path *paths);

protected:
  int tagMatchIndex();
  TagMatchMap *makePathIndexMap(const StaState *sta);

  Vertex *vertex_;
  int default_path_count_;
  TagMatchMap path_index_map_;
  std::vector<Path>  paths_;
  bool has_clk_tag_;
  bool has_genclk_src_tag_;
  bool has_filter_tag_;
  bool has_loop_tag_;
  bool has_propagated_clk_;
  const StaState *sta_;
};

void
pathIndexMapReport(const TagMatchMap *path_index_map,
                   const StaState *sta);

} // namespace
