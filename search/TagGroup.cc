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

#include "TagGroup.hh"

#include <algorithm>

#include "Report.hh"
#include "Debug.hh"
#include "Graph.hh"
#include "PathAnalysisPt.hh"
#include "ClkInfo.hh"
#include "Tag.hh"
#include "Corner.hh"
#include "Search.hh"
#include "Path.hh"

namespace sta {

////////////////////////////////////////////////////////////////

static size_t
tableSizeFor(size_t capacity)
{
  // Load factor at most 1/2.
  size_t size = 16;
  while (size < capacity * 2)
    size *= 2;
  return size;
}

TagMatchMap::TagMatchMap(bool match_crpr_clk_pin,
                         const StaState *sta,
                         size_t capacity) :
  match_crpr_clk_pin_(match_crpr_clk_pin),
  sta_(sta),
  table_(tableSizeFor(capacity), slot_empty),
  size_(0),
  used_(0)
{
  entries_.reserve(capacity);
}

void
TagMatchMap::clear()
{
  entries_.clear();
  std::fill(table_.begin(), table_.end(), slot_empty);
  size_ = 0;
  used_ = 0;
}

int32_t
TagMatchMap::find(Tag *tag,
                  size_t &slot) const
{
  size_t mask = table_.size() - 1;
  size_t i = tag->matchCmpHash(match_crpr_clk_pin_, sta_) & mask;
  bool have_free = false;
  size_t free_slot = 0;
  while (true) {
    int32_t entry = table_[i];
    if (entry == slot_empty) {
      slot = have_free ? free_slot : i;
      return -1;
    }
    if (entry == slot_erased) {
      if (!have_free) {
        have_free = true;
        free_slot = i;
      }
    }
    else {
      Tag *entry_tag = entries_[entry].tag;
      if (entry_tag == tag
          || Tag::matchCmp(entry_tag, tag, match_crpr_clk_pin_, sta_) == 0) {
        slot = i;
        return entry;
      }
    }
    i = (i + 1) & mask;
  }
}

bool
TagMatchMap::hasKey(Tag *tag) const
{
  size_t slot;
  return find(tag, slot) >= 0;
}

void
TagMatchMap::findKey(Tag *tag,
                     size_t &path_index,
                     bool &exists) const
{
  size_t slot;
  int32_t entry = find(tag, slot);
  if (entry >= 0) {
    path_index = entries_[entry].path_index;
    exists = true;
  }
  else
    exists = false;
}

void
TagMatchMap::findKey(Tag *tag,
                     Tag *&map_tag,
                     size_t &path_index,
                     bool &exists) const
{
  size_t slot;
  int32_t entry = find(tag, slot);
  if (entry >= 0) {
    map_tag = entries_[entry].tag;
    path_index = entries_[entry].path_index;
    exists = true;
  }
  else
    exists = false;
}

void
TagMatchMap::insert(Tag *tag,
                    size_t path_index)
{
  if ((used_ + 1) * 2 > table_.size())
    rehash(table_.size() * 2);
  size_t slot;
  int32_t entry = find(tag, slot);
  if (entry >= 0)
    entries_[entry].path_index = path_index;
  else {
    if (table_[slot] == slot_empty)
      used_++;
    table_[slot] = static_cast<int32_t>(entries_.size());
    entries_.push_back({tag, path_index});
    size_++;
  }
}

void
TagMatchMap::replaceKey(Tag *old_tag,
                        Tag *new_tag)
{
  size_t slot;
  int32_t entry = find(old_tag, slot);
  if (entry >= 0)
    entries_[entry].tag = new_tag;
  else
    insert(new_tag, 0);
}

void
TagMatchMap::erase(Tag *tag)
{
  size_t slot;
  int32_t entry = find(tag, slot);
  if (entry >= 0) {
    table_[slot] = slot_erased;
    entries_[entry].tag = nullptr;
    size_--;
  }
}

void
TagMatchMap::rehash(size_t table_size)
{
  table_.assign(table_size, slot_empty);
  size_t mask = table_size - 1;
  for (size_t e = 0; e < entries_.size(); e++) {
    Tag *tag = entries_[e].tag;
    if (tag) {
      size_t i = tag->matchCmpHash(match_crpr_clk_pin_, sta_) & mask;
      while (table_[i] != slot_empty)
        i = (i + 1) & mask;
      table_[i] = static_cast<int32_t>(e);
    }
  }
  used_ = size_;
}

std::vector<TagMatchMap::Entry>
TagMatchMap::sortedEntries() const
{
  std::vector<Entry> sorted;
  sorted.reserve(size_);
  for (const Entry &entry : entries_) {
    if (entry.tag)
      sorted.push_back(entry);
  }
  TagMatchLess less(match_crpr_clk_pin_, sta_);
  std::sort(sorted.begin(), sorted.end(),
            [&less](const Entry &entry1, const Entry &entry2) {
              return less(entry1.tag, entry2.tag);
            });
  return sorted;
}

////////////////////////////////////////////////////////////////

TagGroup::TagGroup(TagGroupIndex index,
		   TagMatchMap *path_index_map,
		   bool has_clk_tag,
		   bool has_genclk_src_tag,
		   bool has_filter_tag,
		   bool has_loop_tag,
		   const StaState *sta) :
  path_index_map_(path_index_map),
  hash_(hash(path_index_map, sta)),
  ref_count_(0),
  index_(index),
  has_clk_tag_(has_clk_tag),
  has_genclk_src_tag_(has_genclk_src_tag),
  has_filter_tag_(has_filter_tag),
  has_loop_tag_(has_loop_tag),
  own_path_map_(true)
{
}

TagGroup::TagGroup(TagGroupBldr *tag_bldr,
		   const StaState *sta) :
  path_index_map_(&tag_bldr->pathIndexMap()),
  hash_(hash(path_index_map_, sta)),
  ref_count_(0),
  own_path_map_(false)
{
}

TagGroup::~TagGroup()
{
  if (own_path_map_)
    delete path_index_map_;
}

void
TagGroup::incrRefCount()
{
  ref_count_++;
}

void
TagGroup::decrRefCount()
{
  ref_count_--;
}

size_t
TagGroup::hash(const TagMatchMap *path_index_map,
	       const StaState *sta)
{
  bool crpr_on = sta->crprActive();
  size_t hash = 0;
  for (auto const [tag, path_index] : *path_index_map)
    hash += tag->hash(crpr_on, sta);
  return hash;
}

bool
TagGroup::hasTag(Tag *tag) const
{
  return path_index_map_->hasKey(tag);
}

size_t
TagGroup::pathIndex(Tag *tag) const
{
  size_t path_index = 0;
  bool exists;
  pathIndex(tag, path_index, exists);
  return path_index;
}

void
TagGroup::pathIndex(Tag *tag,
                    size_t &path_index,
                    bool &exists) const
{
  path_index_map_->findKey(tag, path_index, exists);
}

void
TagGroup::report(const StaState *sta) const
{
  Report *report = sta->report();
  report->reportLine("Group %u hash = %zu", index_, hash_);
  pathIndexMapReport(path_index_map_, sta);
}

void
TagGroup::reportArrivalMap(const StaState *sta) const
{
  pathIndexMapReport(path_index_map_, sta);
}

void
pathIndexMapReport(const TagMatchMap *path_index_map,
                   const StaState *sta)
{
  Report *report = sta->report();
  for (auto const [tag, path_index] : *path_index_map)
    report->reportLine(" %2zu %s",
                       path_index,
                       tag->to_string(sta).c_str());
  report->reportBlankLine();
}

////////////////////////////////////////////////////////////////

TagGroupBldr::TagGroupBldr(bool match_crpr_clk_pin,
			   const StaState *sta) :
  default_path_count_(sta->corners()->count()
                      * RiseFall::index_count
                      * MinMax::index_count),
  path_index_map_(match_crpr_clk_pin, sta, default_path_count_ * 2),
  paths_(default_path_count_),
  has_clk_tag_(false),
  has_genclk_src_tag_(false),
  has_filter_tag_(false),
  has_loop_tag_(false),
  has_propagated_clk_(false),
  sta_(sta)
{
}

bool
TagGroupBldr::empty()
{
  return path_index_map_.empty();
}

void
TagGroupBldr::init(Vertex *vertex)
{
  vertex_ = vertex;
  path_index_map_.clear();
  paths_.clear();
  has_clk_tag_ = false;
  has_genclk_src_tag_ = false;
  has_filter_tag_ = false;
  has_loop_tag_ = false;
}

void
TagGroupBldr::reportArrivalEntries() const
{
  pathIndexMapReport(&path_index_map_, sta_);
}

Path *
TagGroupBldr::tagMatchPath(Tag *tag)
{
  Path *match;
  size_t path_index;
  tagMatchPath(tag, match, path_index);
  return match;
}

void
TagGroupBldr::tagMatchPath(Tag *tag,
                           // Return values.
                           Path *&match,
                           size_t &path_index)
{
  // Find matching group tag.
  // Match is not necessarily equal to original tag because it
  // must only satisfy tagMatch.
  bool exists;
  Tag *tag_match;
  path_index_map_.findKey(tag, tag_match, path_index, exists);
  if (exists)
    match = &paths_[path_index];
  else {
    match = nullptr;
    path_index = 0;
  }
}

Arrival
TagGroupBldr::arrival(size_t path_index) const
{
  return paths_[path_index].arrival();
}

void
TagGroupBldr::setArrival(Tag *tag,
			 const Arrival &arrival)
{
  // Find matching group tag (not necessarily equal to original tag).
  Path *match;
  size_t path_index;
  tagMatchPath(tag, match, path_index);
  setMatchPath(match, path_index, tag, arrival, nullptr, nullptr, nullptr);
}

void
TagGroupBldr::setMatchPath(Path *match,
                           size_t path_index,
                           Tag *tag,
                           Arrival arrival,
                           Path *prev_path,
                           Edge *prev_edge,
                           TimingArc *prev_arc)
{
  if (match) {
    Tag *tag_match = match->tag(sta_);
    // If the tag match exists there has to be a path map entry for it.
    if (tag_match != tag) {
      // Replace tag in arrival map.
      path_index_map_.replaceKey(tag_match, tag);
    }
    paths_[path_index].init(vertex_, tag, arrival, prev_path,
                            prev_edge, prev_arc, sta_);
  }
  else
    insertPath(tag, arrival, prev_path, prev_edge, prev_arc);
}

void
TagGroupBldr::insertPath(Tag *tag,
                         Arrival arrival,
                         Path *prev_path,
                         Edge *prev_edge,
                         TimingArc *prev_arc)

{
  size_t path_index = paths_.size();
  path_index_map_.insert(tag, path_index);
  paths_.emplace_back(vertex_, tag, arrival, prev_path,
                      prev_edge, prev_arc, sta_);

  if (tag->isClock())
    has_clk_tag_ = true;
  if (tag->isGenClkSrcPath())
    has_genclk_src_tag_ = true;
  if (tag->isFilter()
      || tag->clkInfo()->crprPathRefsFilter())
    has_filter_tag_ = true;
  if (tag->isLoop())
    has_loop_tag_ = true;
  if (tag->clkInfo()->isPropagated())
    has_propagated_clk_ = true;
}

void
TagGroupBldr::insertPath(const Path &path)
{
  insertPath(path.tag(sta_), path.arrival(), path.prevPath(),
             path.prevEdge(sta_), path.prevArc(sta_));
}

TagGroup *
TagGroupBldr::makeTagGroup(TagGroupIndex index,
			   const StaState *sta)
{
  return new TagGroup(index, makePathIndexMap(sta),
		      has_clk_tag_, has_genclk_src_tag_, has_filter_tag_,
		      has_loop_tag_, sta);

}

TagMatchMap *
TagGroupBldr::makePathIndexMap(const StaState *sta)
{
  // Path indices in TagMatchLess order, as the ordered map gave them.
  std::vector<TagMatchMap::Entry> sorted = path_index_map_.sortedEntries();
  TagMatchMap *path_index_map = new TagMatchMap(true, sta, sorted.size());
  size_t path_index = 0;
  for (const TagMatchMap::Entry &entry : sorted) {
    path_index_map->insert(entry.tag, path_index);
    path_index++;
  }
  return path_index_map;
}

void
TagGroupBldr::copyPaths(TagGroup *tag_group,
                        Path *paths,
                        bool keep_required)
{
  for (auto const [tag1, path_index1] : path_index_map_) {
    size_t path_index2;
    bool exists2;
    tag_group->pathIndex(tag1, path_index2, exists2);
    if (exists2) {
      if (keep_required) {
        // The builder's paths carry the init required; keep the one found
        // by the last required search instead of forcing a re-propagation.
        Required required = paths[path_index2].required();
        paths[path_index2] = paths_[path_index1];
        paths[path_index2].setRequired(required);
      }
      else
        paths[path_index2] = paths_[path_index1];
    }
    else
      sta_->report()->critical(1351, "tag group missing tag");
  }
}

////////////////////////////////////////////////////////////////

size_t
TagGroupHash::operator()(const TagGroup *group) const
{
  return group->hash();
}

static bool
pathIndexMapEqual(const TagMatchMap *path_index_map1,
                  const TagMatchMap *path_index_map2)
{
  if (path_index_map1->size() == path_index_map2->size()) {
    for (auto const [tag1, path_index1] : *path_index_map1) {
      if (!path_index_map2->hasKey(tag1))
	return false;
    }
    return true;
  }
  else
    return false;
}

bool
TagGroupEqual::operator()(const TagGroup *tag_group1,
			  const TagGroup *tag_group2) const
{
  return tag_group1 == tag_group2
    || (tag_group1->hash() == tag_group2->hash()
	&& pathIndexMapEqual(tag_group1->pathIndexMap(),
                             tag_group2->pathIndexMap()));
}

} // namespace
