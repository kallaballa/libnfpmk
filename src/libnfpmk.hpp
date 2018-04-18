#ifndef NFP_HPP_
#define NFP_HPP_

#include <iostream>
#include <list>
#include <string>
#include <fstream>
#include <streambuf>
#include <vector>
#include <set>
#include <exception>

#include <boost/multiprecision/gmp.hpp>
#include <boost/multiprecision/number.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/util/math.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/io/svg/svg_mapper.hpp>
#include <boost/geometry/geometries/register/point.hpp>

namespace bm = boost::multiprecision;
namespace bg = boost::geometry;
namespace trans = boost::geometry::strategy::transform;


namespace libnfpmk {
#ifdef NFP_DEBUG
#define DEBUG_VAL(x) std::cerr << x << std::endl;
#define DEBUG_MSG(title, value) std::cerr << title << ":" << value << std::endl;
#else
#define DEBUG_VAL(x)
#define DEBUG_MSG(title, value)
#endif

using std::string;

typedef bm::number<bm::gmp_rational, bm::et_off> rational_t;
#ifndef LIBNFP_USE_RATIONAL
typedef long double coord_t;
#else
typedef rational_t coord_t;
#endif

const coord_t MAX_COORD = 999999999999999999;
const coord_t MIN_COORD = std::numeric_limits<coord_t>::min();

class point_t {
public:
	point_t() : x_(0), y_(0) {
	}
	point_t(coord_t x, coord_t y) : x_(x), y_(y) {
	}
	coord_t x_;
	coord_t y_;

	bool operator<(const point_t&  other) const {
      return  this->x_ < other.x_ || (this->x_ == other.x_ && this->y_ < other.y_);
  }
};

const point_t INVALID_POINT = {MAX_COORD, MAX_COORD};
}


BOOST_GEOMETRY_REGISTER_POINT_2D(libnfpmk::point_t, libnfpmk::coord_t, cs::cartesian, x_, y_)


namespace libnfpmk {
	typedef bg::model::segment<point_t> segment_t;
typedef bg::model::polygon<point_t, false, true> polygon_t;
typedef std::vector<polygon_t::ring_type> nfp_t;
typedef bg::model::linestring<point_t> linestring_t;

typedef typename polygon_t::ring_type::size_type psize_t;

void write_svg(std::string const& filename,	typename std::vector<polygon_t> const& polygons, const nfp_t& nfp) {
	polygon_t nfppoly;
	for (const auto& pt : nfp.front()) {
		nfppoly.outer().push_back(pt);
	}

	for (size_t i = 1; i < nfp.size(); ++i) {
		nfppoly.inners().push_back({});
		for (const auto& pt : nfp[i]) {
			nfppoly.inners().back().push_back(pt);
		}
	}
	std::ofstream svg(filename.c_str());

	boost::geometry::svg_mapper<point_t> mapper(svg, 100, 100,	"width=\"200mm\" height=\"200mm\" viewBox=\"-250 -250 500 500\"");
	for (auto p : polygons) {
		mapper.add(p);
		mapper.map(p, "fill-opacity:0.5;fill:rgb(153,204,0);stroke:rgb(153,204,0);stroke-width:2");
	}
	bg::correct(nfppoly);
	mapper.add(nfppoly);
	mapper.map(nfppoly, "fill-opacity:0.5;fill:rgb(204,153,0);stroke:rgb(204,153,0);stroke-width:2");

	for(auto& r: nfppoly.inners()) {
		if(r.size() == 1) {
			mapper.add(r.front());
			mapper.map(r.front(), "fill-opacity:0.5;fill:rgb(204,153,0);stroke:rgb(204,153,0);stroke-width:2");
		} else if(r.size() == 2) {
			segment_t seg(r.front(), *(r.begin()+1));
			mapper.add(seg);
			mapper.map(seg, "fill-opacity:0.5;fill:rgb(204,153,0);stroke:rgb(204,153,0);stroke-width:2");
		}
	}
}


enum Alignment {
	LEFT,
	RIGHT,
	ON
};

Alignment get_alignment(const segment_t& seg, const point_t& pt){
	coord_t res = ((seg.second.x_ - seg.first.x_)*(pt.y_ - seg.first.y_)
			- (seg.second.y_ - seg.first.y_)*(pt.x_ - seg.first.x_));

	if(res == 0) {
		return ON;
	} else	if(res > 0) {
		return LEFT;
	} else {
		return RIGHT;
	}
}
point_t normalize(const point_t& pt) {
	point_t norm = pt;
	coord_t len = bg::length(segment_t{{0,0},pt});

	if(len == 0.0L)
		return {0,0};

	norm.x_ /= len;
	norm.y_ /= len;

	return norm;
}


void read_wkt_polygon(const string& filename, polygon_t& p) {
	std::ifstream t(filename);

	std::string str;
	t.seekg(0, std::ios::end);
	str.reserve(t.tellg());
	t.seekg(0, std::ios::beg);

	str.assign((std::istreambuf_iterator<char>(t)),
							std::istreambuf_iterator<char>());

	str.pop_back();
	bg::read_wkt(str, p);
	bg::correct(p);
}

void removeCoLinear(polygon_t::ring_type& r) {
	assert(r.size() > 2);
	psize_t nextI;
	psize_t prevI = 0;
	segment_t segment(r[r.size() - 2], r[0]);
	polygon_t::ring_type newR;

	for (psize_t i = 1; i < r.size() + 1; ++i) {
		if (i >= r.size())
			nextI = i % r.size() + 1;
		else
			nextI = i;

		if (get_alignment(segment, r[nextI]) != ON) {
			newR.push_back(r[prevI]);
		}
		segment = {segment.second, r[nextI]};
		prevI = nextI;
	}

	r = newR;
}

void removeCoLinear(polygon_t& p) {
	removeCoLinear(p.outer());
	for (auto& r : p.inners())
		removeCoLinear(r);
}

nfp_t generateNFP(polygon_t& pA, polygon_t& pB, const bool checkValidity = true) {
	removeCoLinear(pA);
	removeCoLinear(pB);

	if(checkValidity)  {
		std::string reason;
		if(!bg::is_valid(pA, reason))
			throw std::runtime_error("Polygon A is invalid: " + reason);

		if(!bg::is_valid(pB, reason))
			throw std::runtime_error("Polygon B is invalid: " + reason);
	}

	nfp_t nfp;


#ifdef NFP_DEBUG
  write_svg("nfp.svg", {pA,pB}, nfp);
#endif

  return nfp;
}
}
#endif
