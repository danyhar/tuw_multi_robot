/*
 * Copyright (c) 2017, <copyright holder> <email>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY <copyright holder> <email> ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <copyright holder> <email> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <voronoi_segmentation/segment_expander.h>
#include <ros/ros.h> //DEBUG

namespace voronoi_graph
{
    Segment_Expander::Segment_Expander()
    {
    }

    void Segment_Expander::Initialize(cv::Mat &_map, cv::Mat &_distField, cv::Mat &_voronoiPath)
    {
        nx_ = _map.cols;
        ny_ = _map.rows;
        ns_ = nx_ * ny_;

        distance_field_.reset(new float[nx_ * ny_]);
        voronoi_graph_.reset(new int8_t[nx_ * ny_]);
        global_map_.reset(new int8_t[nx_ * ny_]);

        float *distance_field = distance_field_.get();
        int8_t *voronoi_graph = voronoi_graph_.get();
        int8_t *global_map = global_map_.get();

        for(int i = 0; i < ns_; i++)
        {
            distance_field[i] = ((float*)_distField.data)[i];
            voronoi_graph[i] = ((int8_t*)_voronoiPath.data)[i];
            global_map[i] = ((int8_t*)_map.data)[i];
        }

    }

    void Segment_Expander::optimizeSegments(std::vector<std::shared_ptr<Segment>> &_segments, float _maxPixelsCrossing, float _maxPixelsEndSegment)
    {
        for(int i = 0; i < _segments.size(); i++)
        {
            if((_segments[i]->getStart() - _segments[i]->getEnd()).norm() < _maxPixelsEndSegment && (_segments[i]->GetPredecessors().size() == 0 || _segments[i]->GetSuccessors().size() == 0))
            {
                _segments.erase(_segments.begin() + i);
            }
        }

        for(int i = 0; i < _segments.size(); i++)
        {
            if(!_segments[i]->getOptStart())
            {
                optimizeSegmentsAroundPoint(_segments, _segments[i]->getStart(), _maxPixelsCrossing, i);
            }

            if(!_segments[i]->getOptEnd())
            {
                optimizeSegmentsAroundPoint(_segments, _segments[i]->getEnd(), _maxPixelsCrossing, i);
            }
        }

        for(int i = 0; i < _segments.size(); i++)
        {
            if((_segments[i]->getStart() - _segments[i]->getEnd()).norm() == 0)
            {
                _segments.erase(_segments.begin() + i);
            }
        }
    }


    void Segment_Expander::optimizeSegmentsAroundPoint(std::vector<std::shared_ptr<Segment>> &_segments, const Eigen::Vector2d &pt, float maxPixels, int startIndex)
    {
        for(int i = startIndex; i < _segments.size(); i++)
        {
            if(!_segments[i]->getOptStart())
            {
                if((_segments[i]->getStart() - pt).norm() < maxPixels)
                {
                    _segments[i]->setStart(pt);
                    _segments[i]->getOptStart() = true;
                }
            }

            if(!_segments[i]->getOptEnd())
            {
                if((_segments[i]->getEnd() - pt).norm() < maxPixels)
                {
                    _segments[i]->setEnd(pt);
                    _segments[i]->getOptEnd() = true;
                }
            }
        }
    }


    std::vector< std::shared_ptr<Segment> > Segment_Expander::getGraph(const std::vector<std::vector<Eigen::Vector2d>> &_endPoints, float* _potential,
            float _max_length, float _optimizePixelsCrossing, float _optimizePixelsEndSegments)
    {
        Segment::ResetId();

        std::vector< std::shared_ptr<Segment> > segments;
        std::vector< std::vector< Eigen::Vector2d > >  endPoints = _endPoints;

        std::vector<Crossing> crossings_;

        for(int i = 0; i < endPoints.size(); i++)
        {
            Crossing c(std::vector< Eigen::Vector2d > (endPoints[i]));
            crossings_.push_back(c);
        }

        while(endPoints.size() > 0)
        {
            std::vector<Eigen::Vector2d> startCrossing = endPoints.back();

            if(startCrossing.size() > 0)
            {
                Eigen::Vector2d startPoint = startCrossing.back();

                for(auto it = startCrossing.begin(); it != startCrossing.end(); ++it) //Occupie all Endpoints from the current crossing to avoid short segments
                {
                    Index idx((*it)[0], (*it)[1], nx_, 0, 0, 0);
                    _potential[idx.i] = 0;
                }

                Index start(startPoint[0], startPoint[1], nx_, 0, 0, 0);

                float min_d;
                std::vector<Eigen::Vector2d> path = getPath(start, _potential, endPoints, min_d);

                if(path.size() > 1 && path.size() <= _max_length)
                {
                    std::vector<Eigen::Vector2d> p = std::vector<Eigen::Vector2d>(path);
                    Segment seg(p, min_d);
                    segments.push_back(std::make_shared<Segment>(seg));
                }
                else if(path.size() > 1 && path.size() > _max_length)
                {
                    float nr_paths = std::ceil((float)path.size() / _max_length);
                    float real_length = std::ceil((float)path.size() / nr_paths);

                    std::vector< std::shared_ptr<Segment> > new_segs;

                    for(int i = 0; i < nr_paths; i++)
                    {
                        //Segment
                        std::vector<Eigen::Vector2d> path_n;

                        for(int j = std::max<int>(0, i * real_length - 1); j <= (i + 1)*real_length - 1 && j < path.size(); j++)
                        {
                            path_n.push_back( {path[j][0], path[j][1]});
                        }

                        float mind = getMinD(path_n);

                        Segment nSeg(std::vector<Eigen::Vector2d>(path_n), mind);
                        new_segs.push_back(std::make_shared<Segment>(nSeg));

                        if(i > 0)
                        {
                            new_segs[i - 1]->AddSuccessor(new_segs[i]);
                            new_segs[i]->AddPredecessor(new_segs[i - 1]);
                        }

                        segments.push_back(new_segs[i]);
                    }
                }

                removeEndpoint(start, endPoints);
            }
            else
            {
                endPoints.pop_back();
            }
        }

        for(auto it = segments.begin(); it != segments.end(); ++it)
        {
            bool addedSegment = 0;

            for(auto it_c = crossings_.begin(); it_c != crossings_.end(); ++it_c)
            {
                it_c->TryAddSegment((*it));
            }
        }

        optimizeSegments(segments, _optimizePixelsCrossing, _optimizePixelsEndSegments);
        return segments;
    }

    float Segment_Expander::getMinD(const std::vector< Eigen::Vector2d > &_path)
    {
        Index p1(_path[0][0], _path[0][1], nx_, 0, 0, 0);
        float min_d = distance_field_[p1.i];

        for(auto it = _path.begin(); it != _path.end(); ++it)
        {
            Index p((*it)[0], (*it)[1], nx_, 0, 0, 0);
            min_d = std::min<float>(distance_field_[p.i], min_d);
        }

        return min_d;
    }


    std::vector<std::vector< Eigen::Vector2d > > Segment_Expander::calcEndpoints(float* _potential)
    {
        std::vector<std::vector< Eigen::Vector2d > > endpoints;

        std::fill(_potential, _potential + ns_, -1);

        for(int i = 0; i < ns_; i++)
        {
            //dont examine allready found potentials
            if(_potential[i] >= 0)
                continue;

            //only look for voronoi points
            if(voronoi_graph_[i] >= 0)
                continue;

            if(nrOfNeighbours(i) > 2)
            {
                std::vector< Eigen::Vector2d > crossing = expandCrossing(Index(i, 0, 0, 0), _potential);
                endpoints.push_back(crossing);
            }

        }

        return std::vector<std::vector< Eigen::Vector2d > >(endpoints);
    }

    std::vector< std::pair< Eigen::Vector2d, Eigen::Vector2d > > Segment_Expander::getSegments(const std::vector< std::vector< Eigen::Vector2d > >  &_endPoints, float* _potential)
    {
        std::vector< std::pair< Eigen::Vector2d, Eigen::Vector2d > > segments;
        std::vector< std::vector< Eigen::Vector2d > > endPoints = _endPoints;

        while(endPoints.size() > 0)
        {

            std::vector<Eigen::Vector2d > startCrossing = endPoints.back();

            if(startCrossing.size() > 0)
            {
                Eigen::Vector2d startPoint = startCrossing.back();

                for(auto it = startCrossing.begin(); it != startCrossing.end(); ++it) //Occupie all Endpoints from the current crossing to avoid short segments
                {
                    Index idx((*it)[0], (*it)[1], nx_, 0, 0, 0);
                    _potential[idx.i] = 0;
                }

                Index start(startPoint[0], startPoint[1], nx_, 0, 0, 0);
                Eigen::Vector2d endPoint = expandSegment(start, _potential, endPoints);

                std::pair< Eigen::Vector2d, Eigen::Vector2d > segment(startPoint, endPoint);
                segments.push_back(segment);

                removeEndpoint(start, endPoints);
            }
            else
            {
                endPoints.pop_back();
            }

        }

        return std::vector< std::pair< Eigen::Vector2d, Eigen::Vector2d > >(segments);
    }



    Eigen::Vector2d Segment_Expander::getSegmentNeighbour(const Eigen::Vector2d &_point, const std::vector<std::vector<Eigen::Vector2d>> &_endpoints)
    {
        Index p(_point[0], _point[1], nx_, 0, 0, 0);

        Index next = p.offsetDist(1, 0, nx_, ny_);

        if(checkSegmentPoint(next) && !isEndpoint(next, _endpoints))
            return {next.getX(nx_), next.getY(ny_)};

        next = p.offsetDist(1, 1, nx_, ny_);

        if(checkSegmentPoint(next) && !isEndpoint(next, _endpoints))
            return {next.getX(nx_), next.getY(ny_)};

        next = p.offsetDist(0, 1, nx_, ny_);

        if(checkSegmentPoint(next) && !isEndpoint(next, _endpoints))
            return {next.getX(nx_), next.getY(ny_)};

        next = p.offsetDist(-1, 0, nx_, ny_);

        if(checkSegmentPoint(next) && !isEndpoint(next, _endpoints))
            return {next.getX(nx_), next.getY(ny_)};

        next = p.offsetDist(-1, -1, nx_, ny_);

        if(checkSegmentPoint(next) && !isEndpoint(next, _endpoints))
            return {next.getX(nx_), next.getY(ny_)};

        next = p.offsetDist(0, -1, nx_, ny_);

        if(checkSegmentPoint(next) && !isEndpoint(next, _endpoints))
            return {next.getX(nx_), next.getY(ny_)};

        next = p.offsetDist(1, -1, nx_, ny_);

        if(checkSegmentPoint(next) && !isEndpoint(next, _endpoints))
            return {next.getX(nx_), next.getY(ny_)};

        next = p.offsetDist(-1, 1, nx_, ny_);

        if(checkSegmentPoint(next) && !isEndpoint(next, _endpoints))
            return {next.getX(nx_), next.getY(ny_)};


        ROS_INFO("ERROR");

        return {0, 0}; //Should not happen

    }

    bool Segment_Expander::checkSegmentPoint(const Segment_Expander::Index &_point)
    {
        //Check Boundries
        if(_point.i < 0 || _point.i > ns_)
            return false;


        if(global_map_[_point.i] > 0)
            return false;


        if(voronoi_graph_[_point.i] >= 0)
            return false;

        if(nrOfNeighbours(_point.i) != 2)
            return false;

        return true;
    }



    std::vector< Eigen::Vector2d> Segment_Expander::getPath(const Segment_Expander::Index &_start, float* _potential,
            const std::vector<std::vector<Eigen::Vector2d>> &_endpoints, float& min_d)
    {
        std::vector< Eigen::Vector2d > points;

        int cycle = 0;
        int cycles = ns_;
        _potential[_start.i] = _start.potential;

        Index current(_start.i, _potential[_start.i], 0, _potential[_start.i]);
        min_d = distance_field_[_start.i];
        //clear the queue
        clearpq(queue_);
        queue_.push(current);


        while(!(queue_.empty())  && (cycle < cycles))
        {
            if(queue_.empty())
                break;

            current = queue_.top();
            queue_.pop();

            min_d = std::min<float>(min_d, distance_field_[current.i]);
            points.push_back( {current.getX(nx_), current.getY(nx_)});

            if(current.i != _start.i && isEndpoint(current, _endpoints))
            {

                Eigen::Vector2d point;
                point[0] = current.getX(nx_);
                point[1] = current.getY(nx_);
                return points;
            }
            else
            {
                addExpansionCandidate(current, current.offsetDist(1, 0, nx_, ny_), _potential);
                addExpansionCandidate(current, current.offsetDist(0, 1, nx_, ny_), _potential);
                addExpansionCandidate(current, current.offsetDist(-1, 0, nx_, ny_), _potential);
                addExpansionCandidate(current, current.offsetDist(0, -1, nx_, ny_), _potential);

                addExpansionCandidate(current, current.offsetDist(1, 1, nx_, ny_), _potential);
                addExpansionCandidate(current, current.offsetDist(-1, 1, nx_, ny_), _potential);
                addExpansionCandidate(current, current.offsetDist(-1, -1, nx_, ny_), _potential);
                addExpansionCandidate(current, current.offsetDist(1, -1, nx_, ny_), _potential);
            }

            cycle++;
        }

        float dx = points.front()[0] - points.back()[0];
        float dy = points.front()[1] - points.back()[1];


        if(!findEdgeSegments_ || points.size() <= 3 || std::sqrt(dx * dx + dy * dy) <= 3)
        {
            points.clear();
        }

        return points;
    }


    Eigen::Vector2d Segment_Expander::expandSegment(Segment_Expander::Index _start, float* _potential, const std::vector<std::vector<Eigen::Vector2d>> &_endpoints)
    {
        std::vector<Eigen::Vector2d> points;

        int cycle = 0;
        int cycles = ns_;
        _potential[_start.i] = _start.potential;

        Index current(_start.i, _potential[_start.i], 0, _potential[_start.i]);

        //clear the queue
        clearpq(queue_);
        queue_.push(current);


        while(!(queue_.empty())  && (cycle < cycles))
        {
            if(queue_.empty())
                break;

            current = queue_.top();
            queue_.pop();



            if(current.i != _start.i && isEndpoint(current, _endpoints))
            {

                Eigen::Vector2d point;
                point[0] = current.getX(nx_);
                point[1] = current.getY(nx_);
                return point;
            }
            else
            {
                addExpansionCandidate(current, current.offsetDist(1, 0, nx_, ny_), _potential);
                addExpansionCandidate(current, current.offsetDist(0, 1, nx_, ny_), _potential);
                addExpansionCandidate(current, current.offsetDist(-1, 0, nx_, ny_), _potential);
                addExpansionCandidate(current, current.offsetDist(0, -1, nx_, ny_), _potential);

                addExpansionCandidate(current, current.offsetDist(1, 1, nx_, ny_), _potential);
                addExpansionCandidate(current, current.offsetDist(-1, 1, nx_, ny_), _potential);
                addExpansionCandidate(current, current.offsetDist(-1, -1, nx_, ny_), _potential);
                addExpansionCandidate(current, current.offsetDist(1, -1, nx_, ny_), _potential);
            }

            cycle++;
        }


        if(findEdgeSegments_)
            return Eigen::Vector2d(current.getX(nx_), current.getY(nx_));
        else
            return Eigen::Vector2d(_start.getX(nx_), _start.getY(nx_));

    }



    std::vector< Eigen::Vector2d > Segment_Expander::expandCrossing(const Index &_start, float* _potential)
    {
        std::vector< Eigen::Vector2d > points;

        int cycle = 0;
        int cycles = ns_;
        _potential[_start.i] = _start.potential;

        Index current(_start.i, _potential[_start.i], 0, _potential[_start.i]);

        //clear the queue
        clearpq(queue_);
        queue_.push(current);


        while(!(queue_.empty())  && (cycle < cycles))
        {
            if(queue_.empty())
                break;

            current = queue_.top();
            queue_.pop();

            if(nrOfNeighbours(current.i) <= 2)
            {
                Eigen::Vector2d point;
                point[0] = current.getX(nx_);
                point[1] = current.getY(nx_);
                points.push_back(point);
                _potential[current.i] = -2;           //Reset potential to find point
            }
            else
            {
                addExpansionCandidate(current, current.offsetDist(1, 0, nx_, ny_), _potential);
                addExpansionCandidate(current, current.offsetDist(0, 1, nx_, ny_), _potential);
                addExpansionCandidate(current, current.offsetDist(-1, 0, nx_, ny_), _potential);
                addExpansionCandidate(current, current.offsetDist(0, -1, nx_, ny_), _potential);

                addExpansionCandidate(current, current.offsetDist(1, 1, nx_, ny_), _potential);
                addExpansionCandidate(current, current.offsetDist(-1, 1, nx_, ny_), _potential);
                addExpansionCandidate(current, current.offsetDist(-1, -1, nx_, ny_), _potential);
                addExpansionCandidate(current, current.offsetDist(1, -1, nx_, ny_), _potential);
            }

            cycle++;
        }

        return points;
    }

    void Segment_Expander::addExpansionCandidate(const Index &current, const Index &next, float* potential)
    {
        float potentialPrev = potential[current.i];

        //Check Boundries
        if(next.i < 0 || next.i > ns_)
            return;

        //Dont update allready found potentials
        if(potential[next.i] >= 0)
            return;

        if(global_map_[next.i] > 0)
            return;


        if(voronoi_graph_[next.i] >= 0)
            return;


        float pot = potentialPrev + 1;
        float dist = 0;
        float weight = pot;

        potential[next.i] = pot;

        queue_.push(Index(next.i, weight, dist, pot));
    }


    int Segment_Expander::nrOfNeighbours(int i)
    {
        //0 1 2
        //7   3
        //6 5 4
        int neighbours[8] =
        {
            (voronoi_graph_[(Index(i, 0, 0, 0).offsetDist(-1, 1, nx_, ny_)).i] < 0),
            (voronoi_graph_[(Index(i, 0, 0, 0).offsetDist(0, 1, nx_, ny_)).i] < 0),
            (voronoi_graph_[(Index(i, 0, 0, 0).offsetDist(1, 1, nx_, ny_)).i] < 0),
            (voronoi_graph_[(Index(i, 0, 0, 0).offsetDist(1, 0, nx_, ny_)).i] < 0),
            (voronoi_graph_[(Index(i, 0, 0, 0).offsetDist(1, -1, nx_, ny_)).i] < 0),
            (voronoi_graph_[(Index(i, 0, 0, 0).offsetDist(0, -1, nx_, ny_)).i] < 0),
            (voronoi_graph_[(Index(i, 0, 0, 0).offsetDist(-1, -1, nx_, ny_)).i] < 0),
            (voronoi_graph_[(Index(i, 0, 0, 0).offsetDist(-1, 0, nx_, ny_)).i] < 0)
        };



        //Remove double neighbours e.g.:
        //  x x 0
        //  0 x 0
        //  0 x x
        //should result in 2

        int num_neighbours = neighbours[0] + neighbours[1] + neighbours[2] + neighbours[3] + neighbours[4] + neighbours[5] + neighbours[6] + neighbours[7];

        if(neighbours[0] && neighbours[1] && !neighbours[2])
            num_neighbours--;

        if(!neighbours[0] && neighbours[1] && neighbours[2])
            num_neighbours--;

        if(neighbours[2] && neighbours[3] && !neighbours[4])
            num_neighbours--;

        if(!neighbours[2] && neighbours[3] && neighbours[4])
            num_neighbours--;


        if(neighbours[4] && neighbours[5] && !neighbours[6])
            num_neighbours--;

        if(!neighbours[4] && neighbours[5] && neighbours[6])
            num_neighbours--;


        if(neighbours[6] && neighbours[7] && !neighbours[0])
            num_neighbours--;

        if(!neighbours[6] && neighbours[7] && neighbours[0])
            num_neighbours--;

        return num_neighbours;
    }


    void Segment_Expander::removeEndpoint(Segment_Expander::Index _current, std::vector< std::vector< Eigen::Vector2d > >  &_endpoints)
    {
        for(int j = 0; j < _endpoints.size(); j++)
        {
            if(_endpoints[j].size() > 0)
            {
                for(int i = 0; i < _endpoints[j].size(); i++)
                {
                    if((int)(_endpoints[j][i][0]) == _current.getX(nx_) && (int)(_endpoints[j][i][1]) == _current.getY(nx_))
                    {
                        _endpoints[j].erase(_endpoints[j].begin() + i);

                        if(_endpoints[j].size() == 0)
                        {
                            _endpoints.erase(_endpoints.begin() + j);
                        }
                    }
                }
            }
            else
            {
                _endpoints.erase(_endpoints.begin() + j);
            }
        }
    }


    bool Segment_Expander::isEndpoint(Segment_Expander::Index &_current, const std::vector<std::vector<Eigen::Vector2d>> &_endpoints)
    {
        for(auto it_crossing = _endpoints.begin(); it_crossing != _endpoints.end(); it_crossing++)
        {
            for(auto it = it_crossing->begin(); it != it_crossing->end(); it++)
            {

                if((*it)[0] == _current.getX(nx_) && (*it)[1] == _current.getY(nx_))
                {
                    return true;
                }
            }
        }

        return false;
    }
}