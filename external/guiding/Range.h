/*
    This file is part of the implementation of the SIGGRAPH 2020 paper
    "Robust Fitting of Parallax-Aware Mixtures for Path Guiding",
    as well as the updated implementation of the ACM TOG 2019 paper
    "Volume Path Guiding Based on Zero-Variance Random Walk Theory".
    The implementation extends Mitsuba, a physically based rendering system.

    Copyright (c) 2020 Lukas Ruppert, Sebastian Herholz.

    Mitsuba is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Mitsuba is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef RANGE_H
#define RANGE_H

#include <string>
#include <ostream>
#include <fstream>
#include <vector>
#include <type_traits>
#include <sstream>


#ifndef _MSC_VER
#define __forceinline inline __attribute__((always_inline))
#endif

#define PMM_INLINE __forceinline
#define FINLINE PMM_INLINE

namespace guiding {

/**
 * convenience class for range-based for loops over subsets of containers
 */
template <typename TContainer>
class Range {
public:
    typedef TContainer container_type;
    typedef typename TContainer::value_type value_type;
    typedef typename TContainer::iterator iterator;
private:
    iterator m_begin;
    iterator m_end;

public:
    FINLINE iterator begin() const {
        return m_begin;
    }
    FINLINE iterator end() const {
        return m_end;
    }

    Range() = default;

    Range(TContainer& container) : Range{container.begin(), container.end()} {}
    Range(const iterator& begin, const iterator& end) : m_begin{begin}, m_end{end} {}

    Range(TContainer& container, typename TContainer::difference_type offsetMin, typename TContainer::difference_type count)
        : Range{container.begin(), offsetMin, count} {}
    Range(const iterator& begin, typename TContainer::difference_type offsetMin, typename TContainer::difference_type count)
        : m_begin{begin+offsetMin}, m_end{m_begin+count} {}

    FINLINE value_type& operator[](ptrdiff_t offset) {
        return *(m_begin+offset);
    }

    FINLINE const value_type& operator[](ptrdiff_t offset) const {
        return *(m_begin+offset);
    }

    FINLINE size_t size() const {
        return static_cast<size_t>(std::distance(m_begin, m_end));
    }

    std::string toString() const {
        std::ostringstream oss;
        oss << "Range[size: " << size() << "]\n";
        return oss.str();
    }
};

///to simplify the definition of ranges for arbitrary containers, including other ranges
template<typename TContainer>
class Range<Range<TContainer>> : public Range<TContainer>
{
    template<typename... Ts> Range(Ts... parameters) : Range<TContainer>(parameters...) {}
};

}

#endif // RANGE_H
