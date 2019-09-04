/******************************************************************************
** Copyright (c) 2015, Intel Corporation                                     **
** All rights reserved.                                                      **
**                                                                           **
** Redistribution and use in source and binary forms, with or without        **
** modification, are permitted provided that the following conditions        **
** are met:                                                                  **
** 1. Redistributions of source code must retain the above copyright         **
**    notice, this list of conditions and the following disclaimer.          **
** 2. Redistributions in binary form must reproduce the above copyright      **
**    notice, this list of conditions and the following disclaimer in the    **
**    documentation and/or other materials provided with the distribution.   **
** 3. Neither the name of the copyright holder nor the names of its          **
**    contributors may be used to endorse or promote products derived        **
**    from this software without specific prior written permission.          **
**                                                                           **
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       **
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         **
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR     **
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT      **
** HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,    **
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED  **
** TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR    **
** PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF    **
** LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING      **
** NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS        **
** SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
* ******************************************************************************/
/* Narayanan Sundaram (Intel Corp.)
 * ******************************************************************************/
#include "GraphMatRuntime.h"


class PR {
  public:
    float pagerank;
    int degree;
  public:
    PR() {
      pagerank = 0.3;
      degree = 0;
    }
    int operator!=(const PR& p) {
      return (fabs(p.pagerank-pagerank)>1e-5);
    }
    friend std::ostream &operator<<(std::ostream &outstream, const PR & val)
    {
      outstream << val.pagerank; 
      return outstream;
    }
};

double gComputeTime;

inline void startComputeTime(struct timeval* start) {
  gettimeofday(start, 0);
}

inline void endComputeTime(struct timeval* start, struct timeval* end) {
  gettimeofday(end, 0);
  float time;
  time = (end->tv_sec-start->tv_sec)*1e3+(end->tv_usec-start->tv_usec)*1e-3;
  gComputeTime += time;
}

template<class V, class E=int>
class Degree : public GraphMat::GraphProgram<int, int, V, E> {
  public:

  Degree() {
    this->order = GraphMat::IN_EDGES;
    this->process_message_requires_vertexprop = false;
  }

  bool send_message(const V& vertexprop, int& message) const {
    message = 1;
    return true;
  }

  void process_message(const int& message, const E edge_value, const V& vertexprop, int& result) const {
    result = message;
  }

  void reduce_function(int& a, const int& b) const {
    a += b;
  }

  void apply(const int& message_out, V& vertexprop) {
    vertexprop.degree = message_out; 
  }

};

template <class E>
class PageRank : public GraphMat::GraphProgram<float, float, PR, E> {
  public:
    float alpha;

  public:

  PageRank(float a=0.3) {
    alpha = a;
    this->activity = GraphMat::ALL_VERTICES;
    this->process_message_requires_vertexprop = false;
  }

  void reduce_function(float& a, const float& b) const {
    struct timeval start, end;
    startComputeTime(&start);
    a += b;
    endComputeTime(&start, &end);
  }
  void process_message(const float& message, const E edge_val, const PR& vertexprop, float& res) const {
    struct timeval start, end;
    startComputeTime(&start);
    res = message;
    endComputeTime(&start, &end);
  }
  bool send_message(const PR& vertexprop, float& message) const {
    struct timeval start, end;
    startComputeTime(&start);
    if (vertexprop.degree == 0) {
      message = 0.0;
    } else {
      message = vertexprop.pagerank/(float)vertexprop.degree;
    }
    endComputeTime(&start, &end);
    return true;
  }
  void apply(const float& message_out, PR& vertexprop) {
    struct timeval start, end;
    startComputeTime(&start);
    vertexprop.pagerank = alpha + (1.0-alpha)*message_out; //non-delta update
    endComputeTime(&start, &end);
  }

};


template <class edge>
void run_pagerank(const char* filename) {

  GraphMat::Graph<PR, edge> G;
  PageRank<edge> pr;
  Degree<PR, edge> dg;

 
  G.ReadMTX(filename); 

  auto dg_tmp = GraphMat::graph_program_init(dg, G);

  struct timeval start, end;
  gettimeofday(&start, 0);

  G.setAllActive();
  GraphMat::run_graph_program(&dg, G, 1, &dg_tmp);

  gettimeofday(&end, 0);
  double total_time = (end.tv_sec-start.tv_sec)*1e3+(end.tv_usec-start.tv_usec)*1e-3;
  printf("Degree Time = %.3f ms \n", total_time);

  GraphMat::graph_program_clear(dg_tmp);
  
  auto pr_tmp = GraphMat::graph_program_init(pr, G);

  double graph_time, compute_time;
  gettimeofday(&start, 0);

  G.setAllActive();
  GraphMat::run_graph_program(&pr, G, GraphMat::UNTIL_CONVERGENCE, &pr_tmp);
  
  gettimeofday(&end, 0);
  total_time = (end.tv_sec-start.tv_sec)*1e3+(end.tv_usec-start.tv_usec)*1e-3;
  compute_time = gComputeTime;
  graph_time = total_time - compute_time;
  printf("compute Time = %.3f ms (%.3f percent)\n", compute_time, compute_time/total_time);
  printf("graph Time = %.3f ms (%.3f percent)\n", graph_time, graph_time/total_time);
  printf("Total Time = %.3f ms \n", total_time);
  printf("PR Time = %.3f ms \n", total_time);

  GraphMat::graph_program_clear(pr_tmp);

  MPI_Barrier(MPI_COMM_WORLD);
  for (int i = 1; i <= std::min((unsigned long long int)25, (unsigned long long int)G.getNumberOfVertices()); i++) { 
    if (G.vertexNodeOwner(i)) {
      printf("%d : %d %f\n", i, G.getVertexproperty(i).degree, G.getVertexproperty(i).pagerank);
    }
    fflush(stdout);
    MPI_Barrier(MPI_COMM_WORLD);
  }

}

int main(int argc, char* argv[]) {
  MPI_Init(&argc, &argv);

  const char* input_filename = argv[1];

  if (argc < 2) {
    printf("Correct format: %s A.mtx\n", argv[0]);
    return 0;
  }

  run_pagerank<int>(input_filename);

  MPI_Finalize();
}

