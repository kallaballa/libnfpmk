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

	bool operator==(const point_t& other) const {
		return this->x_ == other.x_ && this->y_ == other.y_;
	}

	bool operator<(const point_t&  other) const {
      return  this->x_ < other.x_ || (this->x_ == other.x_ && this->y_ < other.y_);
  }
};

const point_t INVALID_POINT = {MAX_COORD, MAX_COORD};
}


BOOST_GEOMETRY_REGISTER_POINT_2D(libnfpmk::point_t, libnfpmk::coord_t, cs::cartesian, x_, y_)


namespace libnfpmk {

typedef bg::model::polygon<point_t, false, true> polygon_t;
typedef std::vector<polygon_t::ring_type> nfp_t;
typedef bg::model::linestring<point_t> linestring_t;

typedef typename polygon_t::ring_type::size_type psize_t;

class segment_t: public bg::model::segment<point_t> {
public:
	psize_t index_ = 0;
	coord_t slope_ = 0;
	inline segment_t() {
	}

	inline segment_t(point_t const& p1, point_t const& p2) :
			bg::model::segment<point_t>(p1, p2) {
	}

	bool operator<(const segment_t& other) const {
		return this->slope_ < other.slope_;
	}
};

bg::model::segment<point_t> toBgSegment(segment_t seg) {
	return *dynamic_cast<bg::model::segment<point_t>*>(&seg);
}

std::ostream& operator<<(std::ostream& os, const point_t& pt) {
	os << "{" << pt.x_ << ", " << pt.y_ << "}";
	return os;
}

std::ostream& operator<<(std::ostream& os, const segment_t& seg) {
	os << seg.index_ << '/' << seg.slope_ << ": {" << seg.first << ", " << seg.second << "}";
	return os;
}

//an edge group sorted by slope of segments
class EdgeGroup : public std::set<segment_t> {
public:
	bool positive_;

	EdgeGroup(bool positive = true) : std::set<segment_t>(), positive_ (positive) {
	}
};

void write_svg(std::string const& filename,	typename std::vector<polygon_t> const& polygons, const nfp_t& nfp) {
	std::ofstream svg(filename.c_str());
	boost::geometry::svg_mapper<point_t> mapper(svg, 100, 100,	"width=\"200mm\" height=\"200mm\" viewBox=\"-250 -250 500 500\"");

	if(!nfp.empty()) {
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
		bg::correct(nfppoly);
		mapper.add(nfppoly);
		mapper.map(nfppoly, "fill-opacity:0.5;fill:rgb(204,153,0);stroke:rgb(204,153,0);stroke-width:2");

		for(auto& r: nfppoly.inners()) {
			if(r.size() == 1) {
				mapper.add(r.front());
				mapper.map(r.front(), "fill-opacity:0.5;fill:rgb(204,153,0);stroke:rgb(204,153,0);stroke-width:2");
			} else if(r.size() == 2) {
				segment_t seg(r.front(), *(r.begin()+1));
				mapper.add(toBgSegment(seg));
				mapper.map(toBgSegment(seg), "fill-opacity:0.5;fill:rgb(204,153,0);stroke:rgb(204,153,0);stroke-width:2");
			}
		}
	}
	for (auto p : polygons) {
		mapper.add(p);
		mapper.map(p, "fill-opacity:0.5;fill:rgb(153,204,0);stroke:rgb(153,204,0);stroke-width:2");
	}
}

void write_svg(std::string const& filename,	std::vector<EdgeGroup> const& edgeGroups) {
	std::ofstream svg(filename.c_str());
	boost::geometry::svg_mapper<point_t> mapper(svg, 100, 100,	"width=\"200mm\" height=\"200mm\" viewBox=\"-250 -250 1000 1000\"");


	std::string colorPos = "255,0,0";
	std::string colorNeg = "0,255,0";
	std::string color;
	for (auto g : edgeGroups) {
		if(g.positive_)
			color = colorPos;
		else
			color = colorNeg;

		for(auto seg: g) {
			mapper.add(toBgSegment(seg));
			mapper.map(toBgSegment(seg), "fill-opacity:0.5;fill:rgb(" + color + ");stroke:rgb(" + color + ");stroke-width:2");
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
	segment_t seg{{0,0},pt};
	coord_t len = bg::length(*dynamic_cast<bg::model::segment<point_t>*>(&seg));

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

polygon_t::ring_type invert(const polygon_t::ring_type& r) {
	polygon_t::ring_type ri;
	for(point_t pt : r) {
		ri.push_back(point_t(-pt.x_, -pt.y_));
	}

	return ri;
}

point_t find_minimum_y(const polygon_t::ring_type& r) {
	std::vector<psize_t> result;
	coord_t min = MAX_COORD;

	for(psize_t i = 0; i < r.size() - 1; ++i) {
		if(r[i].y_< min) {
			result.clear();
			min = r[i].y_;
			result.push_back(i);
		} else if (r[i].y_ == min) {
			result.push_back(i);
		}
	}
	return r[result.front()];
}

std::vector<segment_t> makeEdges(polygon_t::ring_type r, const point_t& lowest) {
	r.erase(r.end() - 1);
	psize_t first = 0;
	for(size_t i = 0; i < r.size(); ++i) {
		if(r[i] == lowest) {
			first = i;
			break;
		}
	}

	std::vector<segment_t> edges;
	for(size_t i = first + 1; i < (r.size() + first + 1); ++i) {
		if(i > r.size() - 1) {
			if(i == r.size()) {
				edges.push_back(segment_t(r.back(), r.front()));
			} else {
				edges.push_back(segment_t(r[i - r.size() - 1], r[i - r.size()]));
			}
		} else {
			if(i == 0) {
				edges.push_back(segment_t(r.back(), r.front()));
			} else {
				edges.push_back(segment_t(r[i - 1], r[i]));
			}
		}
	}
	return edges;
}

void labelEdges(std::vector<segment_t>& edges, const point_t& lowest) {
	psize_t first = 0;
	for(size_t i = 0; i < edges.size(); ++i) {
		if(edges[i].first == lowest) {
			first = i;
			break;
		}
	}

	size_t count = 0;
	for(size_t i = first; i < (edges.size() + first); ++i) {
		if(i > edges.size() - 1) {
			edges[i - edges.size()].index_ = count++;
		} else {
			edges[i].index_ = count++;
		}
	}
}

coord_t calculateSlope(const segment_t& seg) {
  coord_t d_x = seg.second.x_ - seg.first.x_;
  coord_t d_y = seg.second.y_ - seg.first.y_;
  return (M_PI - atan2(d_y, -d_x)) / (2 * M_PI) * 360;
}

void calculateSlopes(std::vector<segment_t>& edges) {
	for(segment_t& seg: edges) {
		seg.slope_ = calculateSlope(seg);
	}
}

bool hasSameSign(coord_t x, coord_t y) {
    return (x == 0 && y == 0) || (x > 0 && y > 0) || (x < 0 && y < 0);
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

	polygon_t::ring_type rBCW;
	rBCW.resize(pB.outer().size());
	//make clockwise
	std::reverse_copy(pB.outer().begin(), pB.outer().end(), rBCW.begin());
	//invert coordinates
	polygon_t::ring_type rBCWi = invert(rBCW);
	polygon_t::ring_type rA = pA.outer();
	point_t ptMinYA = find_minimum_y(rA);
	point_t ptMinYBi = find_minimum_y(rBCWi);

	std::vector<segment_t> edgesA = makeEdges(rA, ptMinYA);
	std::vector<segment_t> edgesBi = makeEdges(rBCWi, ptMinYBi);
	std::cerr << "label" << std::endl;

	segment_t seg1{{0,0},{1,0}};
	segment_t seg2{{0,0},{0,1}};
	segment_t seg3{{0,0},{-1,0}};
	segment_t seg4{{0,0},{0,-1}};
	std::cerr << calculateSlope(seg1) << std::endl;
	std::cerr << calculateSlope(seg2) << std::endl;
	std::cerr << calculateSlope(seg3) << std::endl;
	std::cerr << calculateSlope(seg4) << std::endl;
	labelEdges(edgesA, ptMinYA);
	labelEdges(edgesBi, ptMinYBi);
	calculateSlopes(edgesA);
	calculateSlopes(edgesBi);

	for(const segment_t& seg: edgesA) {
		std::cerr << seg << std::endl;
	}
	std::cerr << std::endl;

	for(const segment_t& seg: edgesBi) {
		std::cerr << seg << std::endl;
	}

	std::vector<coord_t> slopDiff;
	coord_t sd;
	for(size_t i = 0; i < edgesBi.size(); ++i) {
		if(i == 0) {
			sd = edgesBi.front().slope_ - edgesBi.back().slope_;
		} else {
			sd = edgesBi[i].slope_ - edgesBi[i - 1].slope_;
		}

		if(sd > 180)
			sd -= 360;
		else if(sd < -180)
			sd += 360;

		slopDiff.push_back(sd);
	}

	std::vector<size_t> turningPoint;

	for(size_t i = 0; i < slopDiff.size(); ++i) {
		if(i == 0) {
			if(!hasSameSign(slopDiff.back(), slopDiff[0])) {
				turningPoint.push_back(i);
			}
		} else {
			if(!hasSameSign(slopDiff[i], slopDiff[i - 1])) {
				turningPoint.push_back(i);
			}
		}
	}

	std::vector<EdgeGroup> edgeGroups;
	bool positive = false;
	if(!turningPoint.empty()) {
		assert(turningPoint.size() > 1);
		for(size_t i = 1; i < turningPoint.size(); ++i) {
			positive = !positive;
			edgeGroups.push_back({positive});
			for(size_t j = turningPoint[i - 1]; j < turningPoint[i]; ++j) {
				edgeGroups.back().insert(edgesBi[j]);
			}
		}
		edgeGroups.push_back({false});
		if(turningPoint.back() <= (edgesBi.size() - 1)) {
			for(size_t i = turningPoint.back(); i < edgesBi.size(); ++i) {
				edgeGroups.back().insert(edgesBi[i]);
			}
		}

		if(turningPoint.front() > 0) {
			for(size_t i = 0; i < turningPoint.front(); ++i) {
				edgeGroups.back().insert(edgesBi[i]);
			}
		}

		if(edgeGroups.back().empty())
			edgeGroups.erase(edgeGroups.end() - 1);
	} else {
		EdgeGroup eg;
		for(auto e : edgesBi)
			eg.insert(e);
		edgeGroups.push_back(eg);
	}

	std::cerr << "num edgegroups: " << edgeGroups.size() << std::endl;
	#ifdef NFP_DEBUG
  write_svg("edgeGroups.svg", edgeGroups);
#endif

	std::cerr << "turning point"  << std::endl;
	for(const size_t& tp : turningPoint) {
		std::cerr << tp << std::endl;
	}
	nfp_t nfp;


#ifdef NFP_DEBUG
  write_svg("nfp.svg", {pA,pB}, nfp);
#endif

  return nfp;
}
}
#endif
