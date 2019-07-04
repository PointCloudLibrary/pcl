#include "internal.h"

#include <pcl/gpu/utils/texture_binder.hpp>
#include <pcl/gpu/utils/device/block.hpp>

#include <cassert>

///////////////////////////////////////////////////////////////////////////////////////
/////////////////////////// Constants /////////////////////////////////////////////////

namespace pcl
{
  namespace device
  {
    struct Edges
    {        
        enum { UP = 1, DOWN = 2, LEFT = 4, RIGHT = 8, INVALID = 0xF0 };                
        
        unsigned char data;
        
        __device__ __host__ __forceinline__ 
        Edges() : data(0) {};        
    };
    
    enum 
    { 
        CTA_SIZE_X = 32, 
        CTA_SIZE_Y = 8, 

        TILES_PER_BLOCK_X = 1,
        TILES_PER_BLOCK_Y = 4,

        TPB_X = TILES_PER_BLOCK_X,
        TPB_Y = TILES_PER_BLOCK_Y,        

        TILE_COLS = CTA_SIZE_X * TPB_X, 
        TILE_ROWS = CTA_SIZE_Y * TPB_Y,

        MERGE_CTA_SIZE = 256
    };
  }
}

///////////////////////////////////////////////////////////////////////////////////////
////////////////////// Edges initialization ///////////////////////////////////////////

namespace pcl
{
  namespace device
  {
    __global__ void 
    fillInvalidEdges(PtrStepSz<Edges> output)
    {
      int x = threadIdx.x + blockIdx.x * blockDim.x;
      int y = threadIdx.y + blockIdx.y * blockDim.y;       

      if( x < output.cols && y < output.rows)
        output.ptr(y)[x].data = Edges::INVALID;
    }
  }
}

void
pcl::device::ConnectedComponents::initEdges(int rows, int cols, DeviceArray2D<unsigned char>& edges)
{
  int ecols = divUp(cols, TILE_COLS) * TILE_COLS;
  int erows = divUp(rows, TILE_ROWS) * TILE_ROWS;

  edges.create(erows, ecols);

  dim3 block(32, 8);
  dim3 grid(divUp(ecols, block.x), divUp(erows, block.y));

  fillInvalidEdges<<<grid, block>>>(edges);
  cudaSafeCall( cudaGetLastError() );
  cudaSafeCall( cudaDeviceSynchronize() );
}

///////////////////////////////////////////////////////////////////////////////////////
///////////////////////// Compute Edges  //////////////////////////////////////////////

namespace pcl
{
  namespace device
  {
    template<typename Point> 
    __device__ __forceinline__ float sqnorm(const Point& p1, const Point& p2) 
    { 
        float dx = (p1.x - p2.x);
        float dy = (p1.y - p2.y);
        float dz = (p1.z - p2.z);
        return dx * dx + dy * dy + dz * dz; 
    }

    __device__ __forceinline__ float3 computePoint(unsigned short depth, int x, int y, const Intr& intr)
    {                  
       float z = depth * 0.001f; // mm -> meters
       float3 result;
       
       result.x = z * (x - intr.cx) / intr.fx;
       result.y = z * (y - intr.cy) / intr.fy;
       result.z = z;
       
       return result;
    }

    __global__ void 
    computeEdgesKernel(const PtrStepSz<unsigned char> labels, const PtrStep<unsigned short> depth, const Intr intr, 
        const int num_parts, const float sq_radius, PtrStep<Edges> output)
    {
      int x = threadIdx.x + blockIdx.x * blockDim.x;
      int y = threadIdx.y + blockIdx.y * blockDim.y;       

      if( x < labels.cols && y < labels.rows)
      {
        unsigned char label = labels.ptr(y)[x];
        
        if (label < num_parts)
        {
          unsigned short d = depth.ptr(y)[x];

          if (d)
          {   
            float3 point = computePoint(d, x, y, intr);
            Edges edges;
            
            if (x > 0)
              if(labels.ptr(y)[x-1] == label)
              {
                float3 pl = computePoint(depth.ptr(y)[x-1], x-1, y, intr);               
                if (sqnorm(pl, point) < sq_radius)              
                  edges.data |= Edges::LEFT;
              }

            if (x < labels.cols - 1)
              if(labels.ptr(y)[x+1] == label)
              {
                float3 pr = computePoint(depth.ptr(y)[x+1], x+1, y, intr);
                if (sqnorm(pr, point) < sq_radius)
                  edges.data |= Edges::RIGHT;
              }

            if (y > 0)
              if(labels.ptr(y-1)[x] == label)
              {
                float3 pu = computePoint(depth.ptr(y-1)[x], x, y-1, intr);
                if(sqnorm(pu, point) < sq_radius)
                  edges.data |= Edges::UP;
              }

            if (y < labels.rows - 1)
              if(labels.ptr(y+1)[x] == label)
              {
                float3 pd = computePoint(depth.ptr(y+1)[x], x, y+1, intr);
                if (sqnorm(pd, point) < sq_radius)
                  edges.data |= Edges::DOWN;    
              }

            output.ptr(y)[x] = edges;
          }
        } /* if (label < num_parts) */
      }
    } /* __global__ */
  }
}

void
pcl::device::ConnectedComponents::computeEdges(const Labels& labels, const Depth& depth, int num_parts, float sq_radius, DeviceArray2D<unsigned char>& edges)
{
  device::Intr intr(525.f, 525.f, 319.5, 239.5);

  initEdges(labels.rows(), labels.cols(), edges);
  
  dim3 block(32, 8);
  dim3 grid(divUp(labels.cols(), block.x), divUp(labels.rows(), block.y));
 
  computeEdgesKernel<<<grid, block>>>(labels, depth, intr, num_parts, sq_radius, edges);
  cudaSafeCall( cudaGetLastError() );
  cudaSafeCall( cudaDeviceSynchronize() );
}

///////////////////////////////////////////////////////////////////////////////////////
///////////// Connected components - shared memory tiles //////////////////////////////

namespace pcl
{
  namespace device
  {
    __global__ void 
    smemTilesKernel(const PtrStepSz<unsigned char> edges, PtrStep<int> comps)
    {
      int x = threadIdx.x + blockIdx.x * TILE_COLS;
      int y = threadIdx.y + blockIdx.y * TILE_ROWS;

      if (x >= edges.cols || y >= edges.rows)
        return;      

      __shared__ int labels_arr[TILE_ROWS][TILE_COLS];
      __shared__ int  edges_arr[TILE_ROWS][TILE_COLS];
                                             

      int new_labels[TPB_Y][TPB_X];
      int old_labels[TPB_Y][TPB_X];

      // load data and compute indeces
      #pragma unroll
      for(int i = 0; i <  TPB_Y; ++i)                
      {
        #pragma unroll
        for(int j = 0; j < TPB_X; ++j)
        {
          int xloc = threadIdx.x+CTA_SIZE_X*j;
          int yloc = threadIdx.y+CTA_SIZE_Y*i;
                      
          int index = yloc * TILE_COLS + xloc;
          unsigned char e = edges.ptr(y + CTA_SIZE_Y*i)[x + CTA_SIZE_X*j];

          //disable edges to prevent out of shared memory access for tile
          //will account such edges on global step
          if (xloc == 0)
              e &= ~Edges::LEFT;

          if (yloc == 0)
              e &= ~Edges::UP;

          if (xloc == CTA_SIZE_X * TPB_X - 1)
              e &= ~Edges::RIGHT;

          if (yloc == CTA_SIZE_Y * TPB_Y - 1)
              e &= ~Edges::DOWN;            

          new_labels[i][j] = index;
          edges_arr[yloc][xloc] = e;
        }
      }                   
              	
      // main loop until no changes in shared memory
	  for(int i = 0; ;++i)
	  {         
        //copy data to shared memory and to old array
        #pragma unroll
        for(int i = 0; i < TPB_Y; ++i)                
        {
          #pragma unroll
          for(int j = 0; j < TPB_X; ++j)
          {
            int xloc = threadIdx.x+CTA_SIZE_X*j;
            int yloc = threadIdx.y+CTA_SIZE_Y*i;
            old_labels[i][j] = new_labels[i][j];
            labels_arr[yloc][xloc] = new_labels[i][j];              
          }
        }
 
        __syncthreads(); 

        // test for neighbours
        #pragma unroll
        for(int i = 0; i < TPB_Y; ++i)                
        {
          #pragma unroll
          for(int j = 0; j < TPB_X; ++j)
          {   
            int xloc = threadIdx.x+CTA_SIZE_X*j;
            int yloc = threadIdx.y+CTA_SIZE_Y*i;
            
            int edges = edges_arr[yloc][xloc];
            int label = new_labels[i][j];                        

            if (edges & Edges::UP)
               label = min(label, labels_arr[yloc-1][xloc]);

            if (edges & Edges::DOWN)
               label = min(label, labels_arr[yloc+1][xloc]);

            if (edges & Edges::LEFT)
               label = min(label, labels_arr[yloc][xloc-1]);

            if (edges & Edges::RIGHT)
               label = min(label, labels_arr[yloc][xloc+1]);

            new_labels[i][j] = label;
          }
        }

        __syncthreads();

        int changed = 0;
        #pragma unroll
        for(int i = 0; i < TPB_Y; ++i)                
        {
          #pragma unroll
          for(int j = 0; j < TPB_X; ++j)
          {
            int old_label = old_labels[i][j];
            int new_label = new_labels[i][j];

            //if there is a neigboring element with a smaller label, update the equivalence tree of the processed element
		    //(the tree is always flattened in this stage so there is no need to use findRoot to find the root)
            if (new_label < old_label)
            {
              changed = 1;                
              atomicMin(&labels_arr[0][0] + old_label, new_label);
            }              
          }
        }

        changed = __syncthreads_or(changed);
        if (!changed)
            break;
       
        //flatten the equivalence tree
        const int *cmem = &labels_arr[0][0];
        #pragma unroll
        for(int i = 0; i < TPB_Y; ++i)                
        {
          #pragma unroll
          for(int j = 0; j < TPB_X; ++j)
          {
            int label = new_labels[i][j];
            
            while( cmem[label] < label )
                label = cmem[label];

            new_labels[i][j] = label;
          }

        }              
	  __syncthreads();
                 				
      }
              
	  //transfer the label into global coordinates and store to global memory
      #pragma unroll
      for(int i = 0; i < TPB_Y; ++i)                
      {
        #pragma unroll
        for(int j = 0; j < TPB_X; ++j)
        {
          int label = new_labels[i][j];
          
          int yloc = label / TILE_COLS;
          int xloc = label - yloc * TILE_COLS;
          
          xloc += blockIdx.x * TILE_COLS;
          yloc += blockIdx.y * TILE_ROWS;

          label = yloc * edges.cols + xloc;
          
          comps.ptr(y + CTA_SIZE_Y*i)[x + CTA_SIZE_X*j] = label;
        }
      }    
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////
///////////////// Connected components - merge tiles //////////////////////////////////

namespace pcl
{
  namespace device
  {   
    __device__ __forceinline__ int findRoot(const PtrStepSz<int>& comps, int label)
    {                       
        for(;;)
        {
          int y = label / comps.cols;
          int x = label - y * comps.cols;

          int parent = comps.ptr(y)[x];

          if (label == parent)
              break;

          label = parent;          
        }
        return label;
    }

    texture<unsigned char, 2, cudaReadModeElementType> edgesTex;
    
    struct TilesMerge
    {           
      int tileSizeX;
      int tileSizeY;
      int tilesNumX;
      int tilesNumY;

      mutable PtrStepSz<int> comps;

      __device__ __forceinline__ void 
      unionF(int l1, int l2, bool& changed) const
      {
        int r1 = findRoot(comps, l1);
        int r2 = findRoot(comps, l2);
        
        if (r1 != r2)
        {          
          int mi = min(r1, r2);
          int ma = max(r1, r2);

          int y = ma / comps.cols;
          int x = ma - y * comps.cols;
          atomicMin(&comps.ptr(y)[x], mi);
          changed = true;
        }        
      } 
      
      __device__ __forceinline__ void
      operator()() const
      {
        int tid = Block::flattenedThreadId();
        int stride = Block::stride();
                
        int xbeg = blockIdx.x * tilesNumX * tileSizeX;
        int xend = xbeg + tilesNumX * tileSizeX;

        int ybeg = blockIdx.y * tilesNumY * tileSizeY;
        int yend = ybeg + tilesNumY * tileSizeY;

        int tasksH = (tilesNumY - 1) * (xend - xbeg);
        int tasksV = (tilesNumX - 1) * (yend - ybeg);

        int total = tasksH + tasksV;
        
        bool changed;
        do
        {           
          changed = false;

          for( int task_index = tid; task_index < total; task_index += stride)
            if (task_index < tasksH)
            {
              int indexH = task_index;

              int row = indexH / (xend - xbeg);
              int col = indexH - row * (xend - xbeg);

              int y = ybeg + (row + 1) * tileSizeY;
              int x = xbeg + col;

              int e = tex2D(edgesTex, x, y);
              if (e & Edges::UP)
              {
                int lc = comps.ptr(y  )[x];
                int lu = comps.ptr(y-1)[x];
                unionF(lc, lu, changed);
              }
            }
            else
            {
              int indexV = task_index - tasksH;

              int col = indexV / (yend - ybeg);
              int row = indexV - col * (yend - ybeg);

              int x = xbeg + (col + 1) * tileSizeX;
              int y = ybeg + row;
              
              int e = tex2D(edgesTex, x, y);
              if (e & Edges::LEFT)
              {
                int lc = comps.ptr(y)[x  ];
                int ll = comps.ptr(y)[x-1];
                unionF(lc, ll, changed);
              }                                                                                 
            }                   
        }
        while(__syncthreads_or(changed));
      }

    };

    __global__ void mergeKernel(const TilesMerge tm) { tm(); }        
  }
}

///////////////////////////////////////////////////////////////////////////////////////
//////////////// Connected components - flatten trees /////////////////////////////////

namespace pcl
{
  namespace device
  {
    __global__ void flattenTreesKernel(PtrStepSz<int> comps, const PtrStep<Edges> edges)
    {
      int x = threadIdx.x + blockIdx.x * blockDim.x;
      int y = threadIdx.y + blockIdx.y * blockDim.y;       

      if( x < comps.cols && y < comps.rows)
      {  
        int invalid = edges.ptr(y)[x].data & Edges::INVALID;                                                
        comps.ptr(y)[x] = invalid ? -1 : findRoot(comps, comps.ptr(y)[x]);
      }             
    }
  }
}

    
void pcl::device::ConnectedComponents::labelComponents(const DeviceArray2D<unsigned char>& edges, DeviceArray2D<int>& comps)
{             
  comps.create(edges.rows(), edges.cols());

  dim3 block(CTA_SIZE_X, CTA_SIZE_Y);
  dim3 grid(divUp(edges.cols(), TILE_COLS), divUp(edges.rows(), TILE_ROWS));
  
  smemTilesKernel<<<grid, block>>>(edges, comps);
  cudaSafeCall( cudaGetLastError() );
  cudaSafeCall( cudaDeviceSynchronize() );

  TextureBinder binder(edges, edgesTex);

  // the code below assumes such resolution
  assert(edges.cols() == 640 && edges.rows() == 480);

  TilesMerge tm;
  tm.comps = comps;

  //merge 4x3 -> 5x5 grid
  tm.tileSizeX = TILE_COLS;
  tm.tileSizeY = TILE_ROWS;
  tm.tilesNumX = 4;
  tm.tilesNumY = 3;

  grid.x = edges.cols()/(tm.tileSizeX * tm.tilesNumX);
  grid.y = edges.rows()/(tm.tileSizeY * tm.tilesNumY);
  
  mergeKernel<<<grid, 768>>>(tm);
  cudaSafeCall( cudaGetLastError() );
  cudaSafeCall( cudaDeviceSynchronize() );

  //merge 5x5 -> 1x1 grid
  tm.tileSizeX = TILE_COLS * tm.tilesNumX;
  tm.tileSizeY = TILE_ROWS * tm.tilesNumY;
  tm.tilesNumX = 5;
  tm.tilesNumY = 5;

  grid.x = edges.cols()/(tm.tileSizeX * tm.tilesNumX); 
  grid.y = edges.rows()/(tm.tileSizeY * tm.tilesNumY); 

  mergeKernel<<<grid, 1024>>>(tm);
  cudaSafeCall( cudaGetLastError() );
  cudaSafeCall( cudaDeviceSynchronize() );

  grid.x = divUp(edges.cols(), block.x);
  grid.y = divUp(edges.rows(), block.y);
  flattenTreesKernel<<<grid, block>>>(comps, edges);
  cudaSafeCall( cudaGetLastError() );
  cudaSafeCall( cudaDeviceSynchronize() );
}
