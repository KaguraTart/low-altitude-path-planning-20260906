#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <queue>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace py = pybind11;
using Vec = std::array<double, 3>;
using RealArray = py::array_t<double, py::array::c_style | py::array::forcecast>;
using IntArray = py::array_t<std::int64_t, py::array::c_style | py::array::forcecast>;
constexpr double EPS = 1e-8;
constexpr double INF = std::numeric_limits<double>::infinity();
double distance(const Vec& a, const Vec& b) {
    return std::hypot(std::hypot(a[0]-b[0], a[1]-b[1]), a[2]-b[2]);
}
void finite(double x, const char* what) {
    if (!std::isfinite(x)) throw std::invalid_argument(std::string(what)+" must be finite");
}
void nonnegative(double x, const char* what) {
    finite(x, what);
    if (x < 0) throw std::invalid_argument(std::string(what)+" must be nonnegative");
}
struct Deadline {
    std::chrono::steady_clock::time_point stop;
    bool enabled;
    explicit Deadline(double ms): enabled(ms > 0) {
        nonnegative(ms, "deadline_ms");
        const auto now=std::chrono::steady_clock::now();
        const auto remaining=std::chrono::steady_clock::time_point::max()-now;
        const auto budget=std::chrono::duration<long double,std::milli>(ms);
        stop=!enabled||budget>=remaining ? std::chrono::steady_clock::time_point::max()
            : now+std::chrono::duration_cast<std::chrono::steady_clock::duration>(budget);
    }
    bool expired() const { return enabled && std::chrono::steady_clock::now() >= stop; }
};
std::vector<Vec> points(const RealArray& arr, const char* label="points") {
    if (arr.ndim()!=2 || arr.shape(1)!=3) throw std::invalid_argument(std::string(label)+" must have shape (N, 3)");
    auto a=arr.unchecked<2>();
    std::vector<Vec> out(arr.shape(0));
    for (py::ssize_t i=0; i<arr.shape(0); ++i) for(int j=0;j<3;++j) {
        finite(a(i,j), label); out[i][j]=a(i,j);
    }
    return out;
}
std::vector<double> numbers(const RealArray& arr, const char* label) {
    if(arr.ndim()!=1) throw std::invalid_argument(std::string(label)+" must be one dimensional");
    std::vector<double> out(arr.size()); auto a=arr.unchecked<1>();
    for(py::ssize_t i=0;i<arr.size();++i) { finite(a(i),label); out[i]=a(i); }
    return out;
}
struct Segment {
    Vec p, q;
    double t0, t1;
    int owner;
    Vec at(double t) const {
        double u=t1>t0 ? std::clamp((t-t0)/(t1-t0),0.,1.) : 0.;
        return {p[0]+u*(q[0]-p[0]),p[1]+u*(q[1]-p[1]),p[2]+u*(q[2]-p[2])};
    }
    Segment shifted(double d) const { return {p,q,t0+d,t1+d,owner}; }
};
struct Trajectory { std::vector<Vec> path; std::vector<double> times; };
std::vector<Trajectory> trajectories(const py::sequence& paths, const py::sequence& times) {
    if(paths.size()!=times.size()) throw std::invalid_argument("paths and times must have equal lengths");
    std::vector<Trajectory> out;
    for(size_t i=0;i<paths.size();++i) {
        auto p=RealArray::ensure(paths[i]); auto t=RealArray::ensure(times[i]);
        if(!p || !t) throw std::invalid_argument("paths and times must contain numeric arrays");
        auto ps=points(p,"path"); auto ts=numbers(t,"times");
        if(ps.empty() || ps.size()!=ts.size()) throw std::invalid_argument("each nonempty path must have one time per point");
        for(size_t j=1;j<ts.size();++j) if(ts[j]<=ts[j-1]) throw std::invalid_argument("trajectory times must be strictly increasing, including hover segments");
        out.push_back({std::move(ps),std::move(ts)});
    }
    return out;
}
std::vector<Segment> segments(const Trajectory& t, int owner, double shift=0) {
    std::vector<Segment> out;
    if(t.path.size()==1) out.push_back({t.path[0],t.path[0],t.times[0]+shift,t.times[0]+shift,owner});
    for(size_t i=1;i<t.path.size();++i) out.push_back({t.path[i-1],t.path[i],t.times[i-1]+shift,t.times[i]+shift,owner});
    return out;
}
struct Collision { bool hit=false; double start=0,end=0,horizontal=0,vertical=0; };
// Exact intersection of a horizontal quadratic disk and a vertical linear slab
// over the shared continuous time interval. Sampling is never used.
Collision collide(const Segment& a, const Segment& b, double hs, double vs) {
    double begin=std::max(a.t0,b.t0), end=std::min(a.t1,b.t1);
    if(begin>end+EPS) return {};
    auto pa=a.at(begin), pb=b.at(begin);
    Vec r{},v{};
    for(int k=0;k<3;++k) {
        r[k]=pa[k]-pb[k];
        v[k]=(a.t1>a.t0?(a.q[k]-a.p[k])/(a.t1-a.t0):0.)-(b.t1>b.t0?(b.q[k]-b.p[k])/(b.t1-b.t0):0.);
    }
    double lo=0,hi=std::max(0.,end-begin);
    double aa=v[0]*v[0]+v[1]*v[1], bb=2*(r[0]*v[0]+r[1]*v[1]);
    double hh=hs+EPS, vv=vs+EPS, cc=r[0]*r[0]+r[1]*r[1]-hh*hh;
    if(aa<1e-24) { if(cc>0) return {}; }
    else {
        double disc=bb*bb-4*aa*cc;
        if(disc<0) return {};
        double sq=std::sqrt(std::max(0.,disc));
        lo=std::max(lo,(-bb-sq)/(2*aa)); hi=std::min(hi,(-bb+sq)/(2*aa));
    }
    if(std::abs(v[2])<1e-15) { if(std::abs(r[2])>vv) return {}; }
    else {
        double z0=(-vv-r[2])/v[2], z1=(vv-r[2])/v[2];
        lo=std::max(lo,std::min(z0,z1));hi=std::min(hi,std::max(z0,z1));
    }
    if(lo>hi+EPS) return {};
    lo=std::max(0.,lo);hi=std::max(lo,hi);
    double mh=aa>1e-24?std::clamp(-(r[0]*v[0]+r[1]*v[1])/aa,lo,hi):lo;
    double mz=std::abs(v[2])>1e-15?std::clamp(-r[2]/v[2],lo,hi):lo;
    return {true,begin+lo,begin+hi,std::hypot(r[0]+mh*v[0],r[1]+mh*v[1]),std::abs(r[2]+mz*v[2])};
}
struct Box { std::array<double,4> lo,hi; };
Box box(const Segment& s) {
    Box b;
    for(int k=0;k<3;++k) {b.lo[k]=std::min(s.p[k],s.q[k]);b.hi[k]=std::max(s.p[k],s.q[k]);}
    b.lo[3]=s.t0;b.hi[3]=s.t1;return b;
}
bool overlap(const Box& a,const Box& b,double h,double v) {
    double margin[4]={h+EPS,h+EPS,v+EPS,EPS};
    for(int k=0;k<4;++k) if(a.lo[k]>b.hi[k]+margin[k] || b.lo[k]>a.hi[k]+margin[k]) return false;
    return true;
}
// A sparse 4-axis BVH stores only occupied segments. There is no dense x/y/z/t array.
struct SegmentIndex {
    struct Node {Box bounds;int left=-1,right=-1;size_t begin=0,end=0;};
    std::vector<Segment> data;
    std::vector<size_t> order;
    std::vector<Node> tree;
    std::vector<int> roots;
    size_t indexed=0;
    int build(size_t begin,size_t end) {
        Box bounds=box(data[order[begin]]);
        for(size_t i=begin+1;i<end;++i) {auto b=box(data[order[i]]);for(int k=0;k<4;++k){bounds.lo[k]=std::min(bounds.lo[k],b.lo[k]);bounds.hi[k]=std::max(bounds.hi[k],b.hi[k]);}}
        int n=static_cast<int>(tree.size());tree.push_back({bounds,-1,-1,begin,end});
        if(end-begin>8) {
            int axis=0;for(int k=1;k<4;++k) if(bounds.hi[k]-bounds.lo[k]>bounds.hi[axis]-bounds.lo[axis]) axis=k;
            size_t mid=(begin+end)/2;
            std::nth_element(order.begin()+begin,order.begin()+mid,order.begin()+end,[&](size_t a,size_t b){auto x=box(data[a]),y=box(data[b]);return x.lo[axis]+x.hi[axis]<y.lo[axis]+y.hi[axis];});
            int left=build(begin,mid),right=build(mid,end);tree[n].left=left;tree[n].right=right;
        }
        return n;
    }
    void rebuild() {
        indexed=data.size();order.resize(indexed);std::iota(order.begin(),order.end(),0);
        tree.clear();roots.clear();if(indexed)roots.push_back(build(0,indexed));
    }
    void add(const std::vector<Segment>& s) {
        if(s.empty())return;
        size_t begin=data.size();data.insert(data.end(),s.begin(),s.end());
        order.resize(data.size());std::iota(order.begin()+begin,order.end(),begin);
        int root=build(begin,data.size());
        // Binary merging builds a logarithmic forest incrementally. Existing
        // segment trees are reused instead of rebuilding all prior trajectories.
        while(!roots.empty()) {
            int left=roots.back();auto a=tree[left],b=tree[root];
            if(a.end-a.begin>2*(b.end-b.begin))break;
            roots.pop_back();Box bounds=a.bounds;
            for(int k=0;k<4;++k){bounds.lo[k]=std::min(bounds.lo[k],b.bounds.lo[k]);bounds.hi[k]=std::max(bounds.hi[k],b.bounds.hi[k]);}
            int parent=static_cast<int>(tree.size());tree.push_back({bounds,left,root,a.begin,b.end});root=parent;
        }
        roots.push_back(root);indexed=data.size();
    }
    template<class Fn> bool query_node(int id,const Box& b,double h,double v,Fn& fn) const {
        const auto& n=tree[id];if(!overlap(n.bounds,b,h,v))return false;
        if(n.left<0){for(size_t i=n.begin;i<n.end;++i){size_t j=order[i];if(overlap(box(data[j]),b,h,v)&&fn(j))return true;}return false;}
        return query_node(n.left,b,h,v,fn)||query_node(n.right,b,h,v,fn);
    }
    template<class Fn> bool query(const Segment& s,double h,double v,Fn fn) const {
        auto b=box(s);
        for(auto root:roots)if(query_node(root,b,h,v,fn))return true;
        for(size_t i=indexed;i<data.size();++i)if(overlap(box(data[i]),b,h,v)&&fn(i))return true;
        return false;
    }
};
struct ConflictRecord {int first,second;Collision collision;};
struct ConflictResult {std::string status="ok";std::vector<ConflictRecord> records;size_t checked=0;};
ConflictResult find_conflicts(const std::vector<Trajectory>& ts,double h,double v,size_t cap,Deadline& deadline) {
    ConflictResult result;SegmentIndex index;
    for(size_t i=0;i<ts.size();++i) {
        if(deadline.expired()){result.status="deadline_exceeded";break;}
        auto ss=segments(ts[i],i);
        for(const auto& s:ss) {
            bool stopped=index.query(s,h,v,[&](size_t j){
                ++result.checked;auto c=collide(s,index.data[j],h,v);
                if(c.hit){result.records.push_back({index.data[j].owner,s.owner,c});if(cap&&result.records.size()>=cap){result.status="conflict_limit";return true;}}
                return false;
            });
            if(stopped) return result;
        }
        index.add(ss);
    }
    return result;
}
// For a fixed pair of linear segments, the forbidden departure times form one
// interval: their joint disk/slab/time feasibility set is convex. Find its upper
// boundary from a known colliding departure, avoiding a coarse time increment.
double advance_departure(double current,double requested) {
    if(!std::isfinite(current)||!std::isfinite(requested))
        throw std::overflow_error("Departure time cannot be represented by a finite number");
    double next=std::max(current+1e-6,requested);
    if(next<=current)next=std::nextafter(current,INF);
    if(!std::isfinite(next)||next<=current)
        throw std::overflow_error("Departure time cannot make representable forward progress");
    return next;
}
double release_delay(const Segment& base,const Segment& reserved,double current,double h,double v) {
    double lo=current,hi=std::max(current,reserved.t1-base.t0)+1e-6;
    if(!std::isfinite(hi))throw std::overflow_error("Reservation release time exceeds finite arithmetic");
    while(hi-lo>1e-8) {
        double mid=lo*.5+hi*.5;
        if(mid<=lo||mid>=hi)break;
        if(collide(base.shifted(mid),reserved,h,v).hit)lo=mid;else hi=mid;
    }
    return advance_departure(hi,hi);
}
struct ScheduleResult {std::string status="ok";std::vector<double> delays;std::vector<std::string> statuses;size_t accepted=0;};
ScheduleResult schedule(const std::vector<Trajectory>& ts,const std::vector<std::array<double,2>>& windows,const std::vector<int>& priorities,double h,double v,size_t fixed_count,Deadline& deadline) {
    ScheduleResult result;result.delays.assign(ts.size(),INF);result.statuses.assign(ts.size(),"unscheduled");
    SegmentIndex index;
    // Fixed inputs describe observed traffic, including inconsistent upstream
    // traffic. Register all of it before considering deadlines or new flights.
    // Existing conflicts must never cause a real obstacle to disappear.
    for(size_t i=0;i<fixed_count;++i) {
        index.add(segments(ts[i],static_cast<int>(i)));
        result.delays[i]=0;
        result.statuses[i]="fixed";
        ++result.accepted;
    }
    std::vector<size_t> order(ts.size()-fixed_count);std::iota(order.begin(),order.end(),fixed_count);
    std::stable_sort(order.begin(),order.end(),[&](size_t a,size_t b){return priorities[a]>priorities[b];});
    for(size_t k=0;k<order.size();++k) {
        if(deadline.expired()){result.status="deadline_exceeded";for(size_t z=k;z<order.size();++z)result.statuses[order[z]]="deadline_exceeded";break;}
        size_t i=order[k];auto ss=segments(ts[i],i);double shift=windows[i][0]-ts[i].times.front();double latest=windows[i][1]-ts[i].times.front();
        bool accepted=false;
        while(shift<=latest+EPS) {
            if(deadline.expired()){result.status="deadline_exceeded";result.statuses[i]="deadline_exceeded";break;}
            bool hit=false;double next=shift;
            for(const auto& base:ss) {
                auto shifted=base.shifted(shift);
                if(index.query(shifted,h,v,[&](size_t j){if(collide(shifted,index.data[j],h,v).hit){next=release_delay(base,index.data[j],shift,h,v);return true;}return false;})){hit=true;break;}
            }
            if(!hit){index.add(segments(ts[i],i,shift));result.delays[i]=shift;result.statuses[i]="scheduled";++result.accepted;accepted=true;break;}
            shift=advance_departure(shift,next);
        }
        if(!accepted && result.statuses[i]=="unscheduled")result.statuses[i]="window_exhausted";
    }
    if(result.status=="ok"&&result.accepted<ts.size())result.status="partial";
    return result;
}
struct Edge {int node;double cost;int id;bool reversed;};
struct Graph {std::vector<Vec> nodes;std::vector<std::vector<Edge>> adjacent;double heuristic_scale=INF;};
Graph graph(const RealArray& nodes,const IntArray& edges,const RealArray& costs,bool directed) {
    Graph g;g.nodes=points(nodes,"nodes");g.adjacent.resize(g.nodes.size());auto c=numbers(costs,"edge costs");
    if(g.nodes.empty())throw std::invalid_argument("graph cannot be empty");
    if(edges.ndim()!=2||edges.shape(1)!=2||static_cast<size_t>(edges.shape(0))!=c.size())throw std::invalid_argument("edges must have shape (M, 2) and one cost per edge");
    auto e=edges.unchecked<2>();
    for(py::ssize_t i=0;i<edges.shape(0);++i){auto a=e(i,0),b=e(i,1);if(a<0||b<0||static_cast<size_t>(a)>=g.nodes.size()||static_cast<size_t>(b)>=g.nodes.size())throw std::invalid_argument("edge node index outside graph");if(c[i]<0)throw std::invalid_argument("edge costs must be nonnegative");g.adjacent[a].push_back({static_cast<int>(b),c[i],static_cast<int>(i),false});if(!directed)g.adjacent[b].push_back({static_cast<int>(a),c[i],static_cast<int>(i),true});double d=distance(g.nodes[a],g.nodes[b]);if(d>0)g.heuristic_scale=std::min(g.heuristic_scale,c[i]/d);}
    if(!std::isfinite(g.heuristic_scale)) g.heuristic_scale=0;
    return g;
}
struct SearchResult {std::string status="no_path";std::vector<int> path;std::vector<double> times;std::vector<int> edge_ids;double cost=INF;size_t expanded=0;};
using QueueItem=std::pair<double,int>;
SearchResult astar(const Graph& g,int start,int goal,const std::vector<int>& blocked,size_t limit,Deadline& deadline) {
    SearchResult r;std::vector<bool> ban(g.nodes.size(),false);for(auto n:blocked){if(n<0||static_cast<size_t>(n)>=ban.size())throw std::invalid_argument("blocked node outside graph");ban[n]=true;}
    if(start<0||goal<0||static_cast<size_t>(start)>=g.nodes.size()||static_cast<size_t>(goal)>=g.nodes.size())throw std::invalid_argument("start or goal node outside graph");
    if(ban[start]||ban[goal])return r;
    std::vector<double> best(g.nodes.size(),INF);std::vector<int> parent(g.nodes.size(),-1);
    std::priority_queue<QueueItem,std::vector<QueueItem>,std::greater<QueueItem>> open;best[start]=0;open.push({g.heuristic_scale*distance(g.nodes[start],g.nodes[goal]),start});
    while(!open.empty()) {
        if(deadline.expired()){r.status="deadline_exceeded";return r;}
        auto [f,n]=open.top();open.pop();if(f>best[n]+g.heuristic_scale*distance(g.nodes[n],g.nodes[goal])+EPS)continue;
        if(n==goal){r.status="ok";r.cost=best[n];for(int x=n;x>=0;x=parent[x])r.path.push_back(x);std::reverse(r.path.begin(),r.path.end());return r;}
        if(limit&&r.expanded>=limit){r.status="iteration_limit";return r;}++r.expanded;
        for(auto e:g.adjacent[n])if(!ban[e.node]&&best[n]+e.cost+EPS<best[e.node]){best[e.node]=best[n]+e.cost;parent[e.node]=n;open.push({best[e.node]+g.heuristic_scale*distance(g.nodes[e.node],g.nodes[goal]),e.node});}
    }return r;
}
using Cell=std::array<std::int64_t,3>;
struct CellHash {size_t operator()(const Cell& c) const noexcept {size_t h=0;for(auto x:c)h^=std::hash<std::int64_t>{}(x)+0x9e3779b9+(h<<6)+(h>>2);return h;}};
using CellSet=std::unordered_set<Cell,CellHash>;
CellSet cells(const IntArray& arr) {
    if(arr.ndim()!=2||arr.shape(1)!=3)
        throw std::invalid_argument("voxel coordinates must have shape (N, 3)");
    auto a=arr.unchecked<2>();
    CellSet out;
    for(py::ssize_t i=0;i<arr.shape(0);++i) out.insert({a(i,0),a(i,1),a(i,2)});
    return out;
}
struct GridResult {std::string status="no_path";std::vector<Vec> path;double cost=INF;size_t expanded=0;};
GridResult grid_astar(Vec start,Vec goal,const std::vector<Vec>& bounds,Vec resolution,const CellSet& blocked,const CellSet& allowed,bool restrict_allowed,size_t limit,Deadline& deadline) {
    if(bounds.size()!=2)throw std::invalid_argument("bounds must have shape (2, 3)");
    Cell shape{},s{},target{};
    for(int k=0;k<3;++k){finite(start[k],"start");finite(goal[k],"goal");finite(resolution[k],"resolution");if(resolution[k]<=0||bounds[1][k]<bounds[0][k])throw std::invalid_argument("resolution must be positive and bounds ordered");double count=std::floor((bounds[1][k]-bounds[0][k])/resolution[k])+1;if(!std::isfinite(count)||count<1||count>=static_cast<double>(std::numeric_limits<std::int64_t>::max()))throw std::overflow_error("grid axis cannot be represented by int64 indices");shape[k]=static_cast<std::int64_t>(count);if(start[k]<bounds[0][k]-EPS||start[k]>bounds[1][k]+EPS||goal[k]<bounds[0][k]-EPS||goal[k]>bounds[1][k]+EPS)throw std::invalid_argument("start and goal must lie inside bounds");s[k]=std::clamp<std::int64_t>(std::llround((start[k]-bounds[0][k])/resolution[k]),0,shape[k]-1);target[k]=std::clamp<std::int64_t>(std::llround((goal[k]-bounds[0][k])/resolution[k]),0,shape[k]-1);}
    auto free=[&](const Cell& c){for(int k=0;k<3;++k)if(c[k]<0||c[k]>=shape[k])return false;return blocked.count(c)==0&&(!restrict_allowed||allowed.count(c));};
    auto position=[&](const Cell& c){return Vec{bounds[0][0]+c[0]*resolution[0],bounds[0][1]+c[1]*resolution[1],bounds[0][2]+c[2]*resolution[2]};};
    GridResult r;if(!free(s)||!free(target))return r;
    struct Item {double f;Cell cell;bool operator>(const Item& other)const{return f>other.f;}};
    std::priority_queue<Item,std::vector<Item>,std::greater<Item>> open;std::unordered_map<Cell,double,CellHash> best;std::unordered_map<Cell,Cell,CellHash> parents;best[s]=0;open.push({distance(position(s),position(target)),s});
    while(!open.empty()) {
        if(deadline.expired()){r.status="deadline_exceeded";return r;}
        auto cur=open.top();open.pop();auto c=cur.cell;if(cur.f>best[c]+distance(position(c),position(target))+EPS)continue;
        if(c==target){std::vector<Vec> reverse;for(Cell x=c;;x=parents.at(x)){reverse.push_back(position(x));if(x==s)break;}r.path.assign(reverse.rbegin(),reverse.rend());if(distance(start,r.path.front())>EPS)r.path.insert(r.path.begin(),start);else r.path.front()=start;if(distance(goal,r.path.back())>EPS)r.path.push_back(goal);else r.path.back()=goal;r.cost=0;for(size_t i=1;i<r.path.size();++i)r.cost+=distance(r.path[i-1],r.path[i]);r.status="ok";return r;}
        if(limit&&r.expanded>=limit){r.status="iteration_limit";return r;}++r.expanded;
        for(int dx=-1;dx<=1;++dx)for(int dy=-1;dy<=1;++dy)for(int dz=-1;dz<=1;++dz){if(dx==0&&dy==0&&dz==0)continue;int delta[3]={dx,dy,dz};Cell next{c[0]+dx,c[1]+dy,c[2]+dz};if(!free(next))continue;bool clear=true;for(int mask=1;mask<8&&clear;++mask){Cell corner=c;bool proper=false;for(int k=0;k<3;++k){if(mask&(1<<k))corner[k]+=delta[k];else if(delta[k])proper=true;}if(proper&&!free(corner))clear=false;}if(!clear)continue;double cand=best[c]+distance(position(c),position(next));auto it=best.find(next);if(it==best.end()||cand+EPS<it->second){best[next]=cand;parents[next]=c;open.push({cand+distance(position(next),position(target)),next});}}
    }return r;
}
using Interval=std::array<double,2>;
std::vector<Interval> safe_intervals(const Vec& p,const SegmentIndex& reservations,double h,double v,double begin,double end) {
    Segment wait{p,p,begin,end,-1};std::vector<Interval> blocks;
    reservations.query(wait,h,v,[&](size_t j){auto c=collide(wait,reservations.data[j],h,v);if(c.hit)blocks.push_back({c.start,c.end});return false;});
    std::sort(blocks.begin(),blocks.end());std::vector<Interval> safe;double cursor=begin;
    for(auto b:blocks){if(b[0]>cursor+1e-6)safe.push_back({cursor,b[0]-1e-6});cursor=std::max(cursor,b[1]+1e-6);}
    if(cursor<=end) safe.push_back({cursor,end});
    return safe;
}
std::vector<bool> wait_mask(const py::object& value,size_t count) {
    std::vector<bool> out(count,true);
    if(value.is_none())return out;
    auto a=py::array::ensure(value);
    if(!a||a.ndim()!=1||static_cast<size_t>(a.size())!=count||!a.dtype().is(py::dtype::of<bool>()))
        throw std::invalid_argument("wait_allowed must contain one boolean per node");
    auto contiguous=py::array_t<bool,py::array::c_style|py::array::forcecast>::ensure(a);
    auto b=contiguous.unchecked<1>();
    for(size_t i=0;i<count;++i)out[i]=b(i);
    return out;
}
// Every orientation has its own relative-time segments. Reverse traversal
// reverses both the geometry and the time law, including unequal sample gaps.
std::vector<std::vector<Segment>> sipp_edges(const Graph& g,const IntArray& edges,const RealArray& durations,const py::object& paths,const py::object& times) {
    if(paths.is_none()!=times.is_none())throw std::invalid_argument("edge_paths and edge_times must be supplied together");
    std::vector<Trajectory> shaped;
    if(!paths.is_none()) {
        if(!py::isinstance<py::sequence>(paths)||!py::isinstance<py::sequence>(times))
            throw std::invalid_argument("edge_paths and edge_times must be sequences");
        shaped=trajectories(paths.cast<py::sequence>(),times.cast<py::sequence>());
        if(shaped.size()!=static_cast<size_t>(edges.shape(0)))throw std::invalid_argument("one edge path and time array is required per edge");
    }
    auto es=edges.unchecked<2>();auto ds=durations.unchecked<1>();
    std::vector<std::vector<Segment>> out(2*edges.shape(0));
    for(py::ssize_t i=0;i<edges.shape(0);++i) {
        if(paths.is_none())out[2*i].push_back({g.nodes[es(i,0)],g.nodes[es(i,1)],0,ds(i),-1});
        else {
            const auto& t=shaped[i];
            if(t.path.size()<2||std::abs(t.times.front())>EPS||std::abs(t.times.back()-ds(i))>EPS)
                throw std::invalid_argument("edge_times must start at zero and end at the positive edge duration");
            if(distance(t.path.front(),g.nodes[es(i,0)])>EPS||distance(t.path.back(),g.nodes[es(i,1)])>EPS)
                throw std::invalid_argument("edge path endpoints must match the edge nodes");
            out[2*i]=segments(t,-1);
        }
        for(auto it=out[2*i].rbegin();it!=out[2*i].rend();++it)
            out[2*i+1].push_back({it->q,it->p,ds(i)-it->t1,ds(i)-it->t0,-1});
    }
    return out;
}
Vec edge_heading(const std::vector<Segment>& edge,bool at_end) {
    for(size_t i=0;i<edge.size();++i) {
        const auto& s=edge[at_end?edge.size()-1-i:i];
        double d=distance(s.p,s.q);
        if(d>EPS)return {(s.q[0]-s.p[0])/d,(s.q[1]-s.p[1])/d,(s.q[2]-s.p[2])/d};
    }
    return {0,0,0};
}
struct SippState {int node,interval,previous;double arrival,depart;int parent,edge_id;};
struct StateKey {int node,interval,previous;bool operator==(const StateKey& b)const{return node==b.node&&interval==b.interval&&previous==b.previous;}};
struct StateHash {size_t operator()(const StateKey& a)const{return (static_cast<size_t>(a.node)*1000003+static_cast<size_t>(a.interval))*1000003+static_cast<size_t>(a.previous+1);}};
SearchResult sipp(const Graph& g,int start,int goal,const std::vector<Trajectory>& occupied,double h,double v,double start_time,double latest,size_t limit,double max_turn,const std::vector<bool>& can_wait,const std::vector<std::vector<Segment>>& edge_segments,Deadline& deadline) {
    if(start<0||goal<0||static_cast<size_t>(start)>=g.nodes.size()||static_cast<size_t>(goal)>=g.nodes.size())
        throw std::invalid_argument("start or goal outside graph");
    finite(start_time,"start_time");
    if(std::isnan(latest)||latest<start_time)
        throw std::invalid_argument("latest_arrival must be at least start_time");
    if(!std::isfinite(max_turn)||max_turn<0||max_turn>3.141592653589793+EPS)throw std::invalid_argument("max_turn_angle must be in [0, pi]");
    SegmentIndex reservations;for(size_t i=0;i<occupied.size();++i){auto ss=segments(occupied[i],i);reservations.data.insert(reservations.data.end(),ss.begin(),ss.end());}reservations.rebuild();
    std::vector<std::vector<Interval>> safe(g.nodes.size());std::vector<bool> ready(g.nodes.size(),false);
    auto intervals=[&](int node)->const std::vector<Interval>&{if(!ready[node]){safe[node]=safe_intervals(g.nodes[node],reservations,h,v,start_time,latest);ready[node]=true;}return safe[node];};
    SearchResult r;std::vector<SippState> states;std::unordered_map<StateKey,std::vector<double>,StateHash> best;std::priority_queue<QueueItem,std::vector<QueueItem>,std::greater<QueueItem>> open;
    // Earliest arrival dominates only when waiting at this node is legal.
    // Otherwise keep distinct arrival labels within the caller's time window.
    auto record=[&](const StateKey& key,double arrival){auto& labels=best[key];if(can_wait[key.node]){if(!labels.empty()&&labels[0]<=arrival+EPS)return false;labels.assign(1,arrival);return true;}auto it=std::lower_bound(labels.begin(),labels.end(),arrival-EPS);if(it!=labels.end()&&*it<=arrival+EPS)return false;labels.insert(it,arrival);return true;};
    const auto& initial=intervals(start);for(size_t i=0;i<initial.size();++i){double t=initial[i][0];states.push_back({start,static_cast<int>(i),-1,t,t,-1,-1});record({start,static_cast<int>(i),-1},t);open.push({t+g.heuristic_scale*distance(g.nodes[start],g.nodes[goal]),static_cast<int>(states.size()-1)});}
    while(!open.empty()) {
        if(deadline.expired()){r.status="deadline_exceeded";return r;}
        auto [f,id]=open.top();open.pop();auto cur=states[id];StateKey key{cur.node,cur.interval,cur.previous};if(can_wait[cur.node]&&cur.arrival>best[key][0]+EPS)continue;
        if(cur.node==goal){std::vector<int> chain;for(int n=id;n>=0;n=states[n].parent)chain.push_back(n);std::reverse(chain.begin(),chain.end());r.path.push_back(start);r.times.push_back(states[chain.front()].arrival);for(size_t z=1;z<chain.size();++z){auto state=states[chain[z]];if(state.depart>r.times.back()+EPS){r.path.push_back(states[state.parent].node);r.times.push_back(state.depart);}r.path.push_back(state.node);r.times.push_back(state.arrival);r.edge_ids.push_back(state.edge_id);}r.cost=cur.arrival-start_time;r.status="ok";return r;}
        if(limit&&r.expanded>=limit){r.status="iteration_limit";return r;}++r.expanded;
        const auto source=intervals(cur.node)[cur.interval];
        for(auto e:g.adjacent[cur.node]) {
            if(e.cost<=0)continue;
            int oriented=2*e.id+static_cast<int>(e.reversed);
            const auto& motion=edge_segments[oriented];
            if(cur.previous>=0&&max_turn<3.141592653589793-EPS){auto a=edge_heading(edge_segments[cur.previous],true),b=edge_heading(motion,false);double al=distance(a,{0,0,0}),bl=distance(b,{0,0,0});if(al>EPS&&bl>EPS){double dot=0;for(int k=0;k<3;++k)dot+=a[k]*b[k];if(std::acos(std::clamp(dot/(al*bl),-1.,1.))>max_turn+EPS)continue;}}
            const auto& dest=intervals(e.node);
            for(size_t j=0;j<dest.size();++j) {
                double departure=can_wait[cur.node]?std::max(cur.arrival,dest[j][0]-e.cost):cur.arrival,last=std::min(source[1],dest[j][1]-e.cost);if(departure>last+EPS||departure+e.cost<dest[j][0])continue;
                bool clear=false;
                while(departure<=last+EPS) {
                    if(deadline.expired()){r.status="deadline_exceeded";return r;}
                    double next=departure;bool hit=false;
                    for(const auto& base:motion){if(deadline.expired()){r.status="deadline_exceeded";return r;}auto edge=base.shifted(departure);if(reservations.query(edge,h,v,[&](size_t k){if(collide(edge,reservations.data[k],h,v).hit){next=release_delay(base,reservations.data[k],departure,h,v);return true;}return false;})){hit=true;break;}}
                    if(!hit){clear=true;break;}if(!can_wait[cur.node])break;departure=advance_departure(departure,next);
                }
                if(!clear) continue;
                double arrival=departure+e.cost;
                int previous=max_turn<3.141592653589793-EPS?oriented:-1;
                StateKey nextkey{e.node,static_cast<int>(j),previous};
                if(!record(nextkey,arrival))continue;
                states.push_back({e.node,static_cast<int>(j),previous,arrival,departure,id,e.id});
                open.push({arrival+g.heuristic_scale*distance(g.nodes[e.node],g.nodes[goal]),static_cast<int>(states.size()-1)});
            }
        }
    }return r;
}
py::dict search_dict(const SearchResult& r){py::dict d;d["status"]=r.status;d["path"]=r.path;d["times"]=r.times;d["edge_ids"]=r.edge_ids;d["cost"]=std::isfinite(r.cost)?py::cast(r.cost):py::none();d["expanded"]=r.expanded;return d;}
struct MotionPrimitive {
    std::string kind;Vec start{},end{},center{};bool has_center=false;
    double length=0,duration=0,v0=0,v1=0,peak=0,accel=0,decel=0,ta=0,tc=0,td=0,radius=0,sweep=0;
};
using Limits=std::unordered_map<std::string,double>;
double limit_value(const Limits& limits,const std::string& key,double fallback=INF) {
    auto i=limits.find(key);return i==limits.end()?fallback:i->second;
}
struct Violation {int primitive;std::string code;double actual,limit;};
struct ValidationResult {std::vector<Violation> violations;Limits maxima;};
void check_max(ValidationResult& out,int i,const std::string& code,double actual,double limit) {
    if(!std::isfinite(actual)||actual>limit+1e-5*std::max(1.,std::abs(limit)))out.violations.push_back({i,code,actual,limit});
}
void check_equal(ValidationResult& out,int i,const std::string& code,double actual,double expected) {
    if(!std::isfinite(actual)||!std::isfinite(expected)||std::abs(actual-expected)>1e-5*std::max({1.,std::abs(actual),std::abs(expected)}))out.violations.push_back({i,code,actual,expected});
}
Vec tangent(const MotionPrimitive& p,bool at_end) {
    if(p.length<=EPS)return {0,0,0};
    if(p.kind!="arc")return {(p.end[0]-p.start[0])/p.length,(p.end[1]-p.start[1])/p.length,(p.end[2]-p.start[2])/p.length};
    const auto& q=at_end?p.end:p.start;double sign=p.sweep>=0?1.:-1.;double horizontal=std::abs(p.sweep)*p.radius/p.length;
    return {-sign*(q[1]-p.center[1])/p.radius*horizontal,sign*(q[0]-p.center[0])/p.radius*horizontal,(p.end[2]-p.start[2])/p.length};
}
ValidationResult validate_motion(const std::vector<MotionPrimitive>& ps,const Limits& limits,const std::vector<double>& starts) {
    ValidationResult out;
    double vmax=limit_value(limits,"max_speed_mps"),climb=limit_value(limits,"climb_speed_mps"),descent=limit_value(limits,"descent_speed_mps");
    double amax=limit_value(limits,"acceleration_mps2"),dmax=limit_value(limits,"deceleration_mps2"),yaw=limit_value(limits,"max_yaw_rate_deg_s")*3.141592653589793/180;
    double rmin=limit_value(limits,"min_turn_radius_m",0),vmin=limit_value(limits,"min_speed_mps",0);
    for(size_t i=0;i<ps.size();++i) {
        const auto& p=ps[i];int id=static_cast<int>(i);
        if(p.kind!="arc"&&p.kind!="line"&&p.kind!="climb"&&p.kind!="descent"&&p.kind!="yaw"&&p.kind!="hover")
            out.violations.push_back({id,"UNKNOWN_PRIMITIVE",0,0});
        if(i){check_max(out,id,"POSITION_DISCONTINUITY",distance(ps[i-1].end,p.start),0);if(!starts.empty())check_equal(out,id,"TIME_DISCONTINUITY",starts[i],starts[i-1]+ps[i-1].duration);}
        for(auto value:{p.length,p.duration,p.v0,p.v1,p.peak,p.accel,p.decel,p.ta,p.tc,p.td,p.radius})if(value<0)out.violations.push_back({id,"NEGATIVE_PARAMETER",value,0});
        if(p.length<=EPS) {
            if(p.kind=="arc")out.violations.push_back({id,"INVALID_ARC_GEOMETRY",p.length,0});
            check_max(out,id,"STATIONARY_DISPLACEMENT",distance(p.start,p.end),0);
            check_max(out,id,"STATIONARY_SPEED",std::max({p.v0,p.v1,p.peak}),0);
            if(p.duration<=0)out.violations.push_back({id,"NONPOSITIVE_DURATION",p.duration,0});
            if(p.kind=="yaw") {
                if(std::abs(p.sweep)<=EPS)out.violations.push_back({id,"MISSING_YAW_SWEEP",0,0});
                check_max(out,id,"YAW_RATE",std::abs(p.sweep)/p.duration,yaw);
                check_max(out,id,"TURN_ANGLE",std::abs(p.sweep)*180/3.141592653589793,limit_value(limits,"max_turn_angle_deg",180));
            }
            if(vmin>0)out.violations.push_back({id,"MIN_SPEED_HOVER",0,vmin});
        } else {
            if(p.kind=="yaw"||p.kind=="hover")out.violations.push_back({id,"STATIONARY_DISPLACEMENT",p.length,0});
            double horizontal=std::hypot(p.end[0]-p.start[0],p.end[1]-p.start[1]);
            if(p.kind=="arc") {
                if(!p.has_center||p.radius<=0||std::abs(p.sweep)<=EPS){out.violations.push_back({id,"INVALID_ARC_GEOMETRY",p.radius,0});continue;}
                check_equal(out,id,"ARC_START_RADIUS",std::hypot(p.start[0]-p.center[0],p.start[1]-p.center[1]),p.radius);
                check_equal(out,id,"ARC_END_RADIUS",std::hypot(p.end[0]-p.center[0],p.end[1]-p.center[1]),p.radius);
                double angle=std::atan2(p.start[1]-p.center[1],p.start[0]-p.center[0])+p.sweep;
                check_max(out,id,"ARC_SWEEP_ENDPOINT",std::hypot(p.end[0]-p.center[0]-p.radius*std::cos(angle),p.end[1]-p.center[1]-p.radius*std::sin(angle)),0);
                if(p.radius+1e-5<rmin)out.violations.push_back({id,"TURN_RADIUS",p.radius,rmin});
                horizontal=std::abs(p.sweep)*p.radius;
            }
            check_equal(out,id,"GEOMETRIC_LENGTH",p.length,std::hypot(horizontal,p.end[2]-p.start[2]));
            check_equal(out,id,"PHASE_DURATION",p.duration,p.ta+p.tc+p.td);
            if(p.duration<=0)out.violations.push_back({id,"NONPOSITIVE_DURATION",p.duration,0});
            check_equal(out,id,"ACCEL_END_SPEED",p.v0+p.accel*p.ta,p.peak);
            check_equal(out,id,"DECEL_END_SPEED",p.peak-p.decel*p.td,p.v1);
            check_equal(out,id,"INTEGRATED_DISTANCE",(p.v0+p.peak)*p.ta*.5+p.peak*p.tc+(p.peak+p.v1)*p.td*.5,p.length);
            check_max(out,id,"PEAK_BELOW_ENDPOINT",std::max(p.v0,p.v1),p.peak);
            double vh=p.peak*horizontal/p.length,vz=p.peak*(p.end[2]-p.start[2])/p.length;
            check_max(out,id,"HORIZONTAL_SPEED",vh,vmax);
            check_max(out,id,"TOTAL_SPEED",p.peak,limit_value(limits,"max_total_speed_mps"));
            check_max(out,id,"CLIMB_SPEED",std::max(0.,vz),climb);check_max(out,id,"DESCENT_SPEED",std::max(0.,-vz),descent);
            if(p.peak+1e-5<vmin)out.violations.push_back({id,"MIN_SPEED",p.peak,vmin});
            if(i&&p.v0+1e-5<vmin)out.violations.push_back({id,"MIN_TRANSITION_SPEED",p.v0,vmin});
            double normal=p.kind=="arc"?vh*vh/p.radius:0;
            // A positive phase still carries its full acceleration even if it
            // is shorter than the geometric comparison tolerance.
            double acceleration=std::hypot(p.ta>0?p.accel:0,normal),deceleration=std::hypot(p.td>0?p.decel:0,normal);
            check_max(out,id,"ACCELERATION",acceleration,amax);check_max(out,id,"DECELERATION",deceleration,dmax);
            double rate=p.kind=="arc"?vh/p.radius:0;check_max(out,id,"YAW_RATE",rate,yaw);
            out.maxima["horizontal_speed_mps"]=std::max(out.maxima["horizontal_speed_mps"],vh);
            out.maxima["climb_speed_mps"]=std::max(out.maxima["climb_speed_mps"],std::max(0.,vz));out.maxima["descent_speed_mps"]=std::max(out.maxima["descent_speed_mps"],std::max(0.,-vz));
            out.maxima["acceleration_mps2"]=std::max(out.maxima["acceleration_mps2"],acceleration);out.maxima["deceleration_mps2"]=std::max(out.maxima["deceleration_mps2"],deceleration);out.maxima["yaw_rate_deg_s"]=std::max(out.maxima["yaw_rate_deg_s"],rate*180/3.141592653589793);
        }
        if(i) {
            auto a=tangent(ps[i-1],true),b=tangent(p,false);Vec diff{};
            for(int k=0;k<3;++k)diff[k]=ps[i-1].v1*a[k]-p.v0*b[k];
            check_max(out,id,"VELOCITY_DISCONTINUITY",std::hypot(std::hypot(diff[0],diff[1]),diff[2]),0);
        }
    }
    int previous=-1;
    double continuous_arc_sweep=0;
    for(size_t i=0;i<ps.size();++i) {
        if(ps[i].length<=EPS)continue;
        if(ps[i].kind=="arc") {
            // Splitting one continuous turn into several analytic records
            // must not lower the certified physical turning angle.
            bool continuation=previous>=0&&ps[previous].kind=="arc"&&
                ps[previous].sweep*ps[i].sweep>0&&
                distance(tangent(ps[previous],true),tangent(ps[i],false))<=1e-5;
            continuous_arc_sweep=continuation?continuous_arc_sweep+ps[i].sweep:ps[i].sweep;
            check_max(out,static_cast<int>(i),"TURN_ANGLE",std::abs(continuous_arc_sweep)*180/3.141592653589793,limit_value(limits,"max_turn_angle_deg",180));
        } else continuous_arc_sweep=0;
        if(previous>=0) {
            auto a=tangent(ps[previous],true),b=tangent(ps[i],false);
            double ah=std::hypot(a[0],a[1]),bh=std::hypot(b[0],b[1]);
            double horizontal_angle=0;
            if(ah>EPS&&bh>EPS)horizontal_angle=std::atan2(a[0]*b[1]-a[1]*b[0],a[0]*b[0]+a[1]*b[1]);
            double tangent_difference=distance(a,b);
            double sweeps=0;bool has_yaw=false;
            for(size_t j=static_cast<size_t>(previous)+1;j<i;++j)if(ps[j].kind=="yaw"){sweeps+=ps[j].sweep;has_yaw=true;}
            if(rmin>0&&tangent_difference>1e-5)
                out.violations.push_back({static_cast<int>(i),"TURN_RADIUS_BYPASS",tangent_difference,0});
            if(has_yaw) {
                check_max(out,static_cast<int>(i),"YAW_HEADING_MISMATCH",std::abs(std::remainder(sweeps-horizontal_angle,2*3.141592653589793)),0);
                check_max(out,static_cast<int>(i),"TURN_ANGLE",std::abs(sweeps)*180/3.141592653589793,limit_value(limits,"max_turn_angle_deg",180));
                if(rmin>0&&std::abs(sweeps)>EPS)out.violations.push_back({static_cast<int>(i),"TURN_RADIUS_HOVER_BYPASS",0,rmin});
            } else if(rmin<=0&&std::abs(horizontal_angle)>1e-5&&std::isfinite(yaw))
                out.violations.push_back({static_cast<int>(i),"MISSING_YAW_TRANSITION",std::abs(horizontal_angle),0});
            check_max(out,static_cast<int>(i),"TURN_ANGLE",std::abs(horizontal_angle)*180/3.141592653589793,limit_value(limits,"max_turn_angle_deg",180));
        }
        previous=static_cast<int>(i);
    }
    return out;
}
std::vector<MotionPrimitive> motion_primitives(const py::sequence& values) {
    std::vector<MotionPrimitive> out;
    for(auto value:values) {
        auto d=py::cast<py::dict>(value);MotionPrimitive p;p.kind=py::cast<std::string>(d["kind"]);p.start=py::cast<Vec>(d["start"]);p.end=py::cast<Vec>(d["end"]);
        for(double x:p.start) finite(x,"primitive start");
        for(double x:p.end) finite(x,"primitive end");
        if(d.contains("center")&&!d["center"].is_none()){p.center=py::cast<Vec>(d["center"]);p.has_center=true;for(double x:p.center)finite(x,"primitive center");}
        auto get=[&](const char* key){double result=d.contains(key)?py::cast<double>(d[key]):0;finite(result,key);return result;};
        p.length=get("length");p.duration=get("duration");p.v0=get("v0");p.v1=get("v1");p.peak=get("peak");p.accel=get("accel");p.decel=get("decel");p.ta=get("ta");p.tc=get("tc");p.td=get("td");p.radius=get("radius");p.sweep=get("sweep");out.push_back(p);
    }return out;
}
py::dict validation_dict(const ValidationResult& r) {
    py::list violations;for(auto v:r.violations){py::dict d;d["primitive"]=v.primitive;d["code"]=v.code;d["actual"]=std::isfinite(v.actual)?py::cast(v.actual):py::none();d["limit"]=std::isfinite(v.limit)?py::cast(v.limit):py::none();violations.append(d);}
    py::dict d;d["valid"]=r.violations.empty();d["violations"]=violations;d["maxima"]=r.maxima;return d;
}
ValidationResult validate_samples(const Trajectory& t,const Limits& limits) {
    ValidationResult out;std::vector<Vec> velocities;std::vector<double> durations;
    for(size_t i=1;i<t.path.size();++i) {
        double dt=t.times[i]-t.times[i-1];Vec v{};for(int k=0;k<3;++k)v[k]=(t.path[i][k]-t.path[i-1][k])/dt;
        double vh=std::hypot(v[0],v[1]);
        check_max(out,static_cast<int>(i-1),"HORIZONTAL_SPEED",vh,limit_value(limits,"max_speed_mps"));
        check_max(out,static_cast<int>(i-1),"CLIMB_SPEED",std::max(0.,v[2]),limit_value(limits,"climb_speed_mps"));
        check_max(out,static_cast<int>(i-1),"DESCENT_SPEED",std::max(0.,-v[2]),limit_value(limits,"descent_speed_mps"));
        out.maxima["horizontal_speed_mps"]=std::max(out.maxima["horizontal_speed_mps"],vh);velocities.push_back(v);durations.push_back(dt);
    }
    for(size_t i=1;i<velocities.size();++i) {
        auto a=velocities[i-1],b=velocities[i];double dt=(durations[i-1]+durations[i])*.5;
        double amag=std::hypot(std::hypot(b[0]-a[0],b[1]-a[1]),b[2]-a[2])/dt;
        double oldspeed=std::hypot(std::hypot(a[0],a[1]),a[2]),newspeed=std::hypot(std::hypot(b[0],b[1]),b[2]);
        check_max(out,static_cast<int>(i),newspeed>=oldspeed?"ACCELERATION":"DECELERATION",amag,limit_value(limits,newspeed>=oldspeed?"acceleration_mps2":"deceleration_mps2"));
        const auto& p=t.path[i-1];const auto& q=t.path[i];const auto& r=t.path[i+1];
        double ux=q[0]-p[0],uy=q[1]-p[1],vx=r[0]-q[0],vy=r[1]-q[1];
        double u=std::hypot(ux,uy),v=std::hypot(vx,vy),w=std::hypot(r[0]-p[0],r[1]-p[1]),cross=std::abs(ux*vy-uy*vx);
        if(u>EPS&&v>EPS) {
            double angle=std::atan2(cross,ux*vx+uy*vy),radius=INF;
            if(cross>1e-12*u*v)radius=u*v*w/(2*cross);else if(ux*vx+uy*vy<0)radius=0;
            double rmin=limit_value(limits,"min_turn_radius_m",0);
            if(radius+1e-5<rmin)out.violations.push_back({static_cast<int>(i),"TURN_RADIUS",radius,rmin});
            check_max(out,static_cast<int>(i),"TURN_ANGLE",angle*180/3.141592653589793,limit_value(limits,"max_turn_angle_deg",180));
            check_max(out,static_cast<int>(i),"YAW_RATE",angle/dt*180/3.141592653589793,limit_value(limits,"max_yaw_rate_deg_s"));
            if(std::isfinite(radius)&&radius>0) {
                auto found=out.maxima.find("curvature_per_m");double curvature=1/radius;
                if(found==out.maxima.end()||curvature>found->second)out.maxima["curvature_per_m"]=curvature;
            }
        }
    }
    return out;
}
PYBIND11_MODULE(_native,m) {
    m.doc()="C++17 continuous conflict checks, sparse reservations, A* and safe interval path planning";
    m.attr("ENGINE_VERSION")="2.0.0";
    m.def("validate_motion",[](const py::sequence& primitives,const Limits& limits,const std::vector<double>& starts){auto ps=motion_primitives(primitives);if(!starts.empty()&&starts.size()!=ps.size())throw std::invalid_argument("starts must contain one timestamp per primitive");for(auto t:starts)finite(t,"primitive start time");for(const auto& entry:limits){nonnegative(entry.second,entry.first.c_str());}ValidationResult r;{py::gil_scoped_release release;r=validate_motion(ps,limits,starts);}return validation_dict(r);},py::arg("primitives"),py::arg("limits"),py::arg("starts")=std::vector<double>{});
    m.def("validate_trajectory",[](const RealArray& path,const RealArray& times,const Limits& limits){auto p=points(path,"path");auto t=numbers(times,"times");if(p.empty()||p.size()!=t.size())throw std::invalid_argument("path must have one timestamp per point");for(size_t i=1;i<t.size();++i)if(t[i]<=t[i-1])throw std::invalid_argument("times must be strictly increasing");for(const auto& entry:limits)nonnegative(entry.second,entry.first.c_str());ValidationResult r;{py::gil_scoped_release release;r=validate_samples({p,t},limits);}return validation_dict(r);},py::arg("path"),py::arg("times"),py::arg("limits"));
    m.def("graph_search",[](const RealArray& ns,const IntArray& es,const RealArray& cs,int start,int goal,const std::vector<int>& blocked,size_t limit,double ms,bool directed){auto g=graph(ns,es,cs,directed);SearchResult r;{py::gil_scoped_release release;Deadline deadline(ms);r=astar(g,start,goal,blocked,limit,deadline);}return search_dict(r);},py::arg("nodes"),py::arg("edges"),py::arg("edge_costs"),py::arg("start"),py::arg("goal"),py::arg("blocked_nodes")=std::vector<int>{},py::arg("max_iterations")=0,py::arg("deadline_ms")=0.,py::arg("directed")=false);
    m.def("grid_search",[](Vec start,Vec goal,const RealArray& bounds,Vec resolution,const IntArray& blocked,const py::object& allowed,size_t limit,double ms){auto bs=points(bounds,"bounds");auto blocked_set=cells(blocked);CellSet allowed_set;bool restrict_allowed=!allowed.is_none();if(restrict_allowed)allowed_set=cells(IntArray::ensure(allowed));GridResult r;{py::gil_scoped_release release;Deadline deadline(ms);r=grid_astar(start,goal,bs,resolution,blocked_set,allowed_set,restrict_allowed,limit,deadline);}py::dict d;d["status"]=r.status;d["path"]=r.path;d["cost"]=std::isfinite(r.cost)?py::cast(r.cost):py::none();d["expanded"]=r.expanded;return d;},py::arg("start"),py::arg("goal"),py::arg("bounds"),py::arg("resolution"),py::arg("blocked"),py::arg("allowed")=py::none(),py::arg("max_iterations")=0,py::arg("deadline_ms")=0.);
    m.def("continuous_conflicts",[](const py::sequence& paths,const py::sequence& times,double h,double v,size_t cap,double ms){nonnegative(h,"horizontal_separation");nonnegative(v,"vertical_separation");auto ts=trajectories(paths,times);ConflictResult r;{py::gil_scoped_release release;Deadline deadline(ms);r=find_conflicts(ts,h,v,cap,deadline);}py::list records;for(auto rec:r.records){py::dict d;d["first"]=rec.first;d["second"]=rec.second;d["start"]=rec.collision.start;d["end"]=rec.collision.end;d["min_horizontal"]=rec.collision.horizontal;d["min_vertical"]=rec.collision.vertical;records.append(d);}py::dict d;d["status"]=r.status;d["conflicts"]=records;d["checked_pairs"]=r.checked;return d;},py::arg("paths"),py::arg("times"),py::arg("horizontal_separation"),py::arg("vertical_separation"),py::arg("max_conflicts")=0,py::arg("deadline_ms")=0.);
    m.def("schedule_paths",[](const py::sequence& paths,const py::sequence& times,const RealArray& windows,double h,double v,const py::object& priority_arg,double ms,size_t fixed_count){nonnegative(h,"horizontal_separation");nonnegative(v,"vertical_separation");auto ts=trajectories(paths,times);if(fixed_count>ts.size())throw std::invalid_argument("fixed_count cannot exceed the number of trajectories");if(windows.ndim()!=2||windows.shape(1)!=2||static_cast<size_t>(windows.shape(0))!=ts.size())throw std::invalid_argument("departure_windows must have shape (N, 2)");auto w=windows.unchecked<2>();std::vector<std::array<double,2>> ws(ts.size());for(size_t i=0;i<ts.size();++i){finite(w(i,0),"departure_windows");finite(w(i,1),"departure_windows");if(w(i,0)>w(i,1))throw std::invalid_argument("departure windows must be ordered");ws[i]={w(i,0),w(i,1)};}std::vector<int> priorities(ts.size(),0);if(!priority_arg.is_none()){priorities=priority_arg.cast<std::vector<int>>();if(priorities.size()!=ts.size())throw std::invalid_argument("one priority is required per path");}ScheduleResult r;{py::gil_scoped_release release;Deadline deadline(ms);r=schedule(ts,ws,priorities,h,v,fixed_count,deadline);}py::list delays;for(auto x:r.delays)delays.append(std::isfinite(x)?py::cast(x):py::none());py::dict d;d["status"]=r.status;d["delays"]=delays;d["statuses"]=r.statuses;d["accepted"]=r.accepted;return d;},py::arg("paths"),py::arg("times"),py::arg("departure_windows"),py::arg("horizontal_separation"),py::arg("vertical_separation"),py::arg("priorities")=py::none(),py::arg("deadline_ms")=0.,py::arg("fixed_count")=0);
    m.def("solve_sipp",[](const RealArray& ns,const IntArray& es,const RealArray& ds,int start,int goal,const py::sequence& paths,const py::sequence& times,double h,double v,double begin,double latest,size_t limit,double ms,bool directed,double turn,const py::object& wait_allowed,const py::object& edge_paths,const py::object& edge_times){nonnegative(h,"horizontal_separation");nonnegative(v,"vertical_separation");auto g=graph(ns,es,ds,directed);auto allowed=wait_mask(wait_allowed,g.nodes.size());auto shaped=sipp_edges(g,es,ds,edge_paths,edge_times);auto occupied=trajectories(paths,times);SearchResult r;{py::gil_scoped_release release;Deadline deadline(ms);r=sipp(g,start,goal,occupied,h,v,begin,latest,limit,turn,allowed,shaped,deadline);}return search_dict(r);},py::arg("nodes"),py::arg("edges"),py::arg("durations"),py::arg("start"),py::arg("goal"),py::arg("reservations_paths")=py::list(),py::arg("reservations_times")=py::list(),py::arg("horizontal_separation")=0.,py::arg("vertical_separation")=0.,py::arg("start_time")=0.,py::arg("latest_arrival")=INF,py::arg("max_iterations")=0,py::arg("deadline_ms")=0.,py::arg("directed")=false,py::arg("max_turn_angle")=3.141592653589793,py::arg("wait_allowed")=py::none(),py::arg("edge_paths")=py::none(),py::arg("edge_times")=py::none());
}
