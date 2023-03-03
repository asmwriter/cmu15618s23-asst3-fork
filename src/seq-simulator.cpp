#include "quad-tree.h"
#include "world.h"
#include <algorithm>
#include <iostream>

// TASK 1

// NOTE: You may modify any of the contents of this file, but preserve all
// function types and names. You may add new functions if you believe they will
// be helpful.

const int QuadTreeLeafSize = 8;
class SequentialNBodySimulator : public INBodySimulator {
public:

  std::unique_ptr<QuadTreeNode> createQuadTreeNode(std::vector<Particle> &particles){
    std::unique_ptr<QuadTreeNode> QNode = std::make_unique<QuadTreeNode>();
    QNode->particles = particles;
    return QNode;
  }

  void helper(std::unique_ptr<QuadTreeNode>& parentNode, std::vector<Particle> &particles, Vec2 bmin, Vec2 bmax, int level){
    std::cout<<"Level:"<<level<<std::endl;
    std::cout<<"QuadTreeLeafSize = "<< QuadTreeLeafSize <<", particles size = "<<particles.size()<<std::endl;
    std::cout<<"Bbox X min: "<<bmin.x<<", max: "<<bmax.x<<std::endl;
    std::cout<<"Bbox Y min: "<<bmin.y<<", max: "<<bmax.y<<std::endl;
    //Update Total mass and bounds for this quadrant
    parentNode->totalMass = 0; 
    parentNode->bmax = bmax;
    parentNode->bmin = bmin;
    
      parentNode->isLeaf = (particles.size() == 1) ? 1:0;
      
      Vec2 pivot = (bmin + bmax) * 0.5f;
      Vec2 size = (bmax - bmin) * 0.5f;

      Vec2 topLeftBmin = Vec2(bmin.x, pivot.y);
      Vec2 topLeftBmax = Vec2(pivot.x, bmax.y);
      Vec2 topRightBmin = Vec2(pivot.x, pivot.y);
      Vec2 topRightBmax = Vec2(bmax.x, bmax.y);
      Vec2 botLeftBmin = Vec2(bmin.x, bmin.y);
      Vec2 botLeftBmax = Vec2(pivot.x, pivot.y);
      Vec2 botRightBmin = Vec2(pivot.x, bmin.y);
      Vec2 botRightBmax = Vec2(bmax.x, pivot.y);

      std::vector<Particle> topLeftQ, topRightQ, botLeftQ, botRightQ;

      //Insert children nodes
      Vec2 weightedPos = 0.0;
      for(auto &particle: particles){
        parentNode->totalMass += particle.mass;
        weightedPos += particle.position * particle.mass;
        //std::cout<<"particle id:"<<particle.id<<", x:"<<particle.position.x<<", y:"<<particle.position.y<<std::endl;
        if(particle.position.x >= topLeftBmin.x && particle.position.x <= topLeftBmax.x &&
             particle.position.y >= topLeftBmin.y && particle.position.y <= topLeftBmax.y){
              // std::cout<<"top left"<<std::endl;
              topLeftQ.push_back(particle);
        }
        if(particle.position.x >= topRightBmin.x && particle.position.x <= topRightBmax.x &&
             particle.position.y >= topRightBmin.y && particle.position.y <= topRightBmax.y){
              // std::cout<<"top right"<<std::endl;
              topRightQ.push_back(particle);
        }
        if(particle.position.x >= botLeftBmin.x && particle.position.x <= botLeftBmax.x &&
             particle.position.y >= botLeftBmin.y && particle.position.y <= botLeftBmax.y){
              // std::cout<<"bottom left"<<std::endl;
              botLeftQ.push_back(particle);
        }
        if(particle.position.x >= botRightBmin.x && particle.position.x <= botRightBmax.x &&
             particle.position.y >= botRightBmin.y && particle.position.y <= botRightBmax.y){
              // std::cout<<"bottom right"<<std::endl;
              botRightQ.push_back(particle);
        }
      }
      //Calculate centre of mass
      parentNode->centreofmass = weightedPos * (1/parentNode->totalMass);
      // std::cout<<"centre of mass - x:"<<parentNode->centreofmass.x<<", y:"<<parentNode->centreofmass.y<<std::endl;
      
      //Create children nodes
      parentNode->children[0] =  std::make_unique<QuadTreeNode>(); 
      parentNode->children[0]->particles = botLeftQ;

      
      parentNode->children[1] =  std::make_unique<QuadTreeNode>(); 
      parentNode->children[1]->particles = botRightQ;


      parentNode->children[2] =  std::make_unique<QuadTreeNode>(); 
      parentNode->children[2]->particles = topLeftQ;

      parentNode->children[3] =  std::make_unique<QuadTreeNode>(); 
      parentNode->children[3]->particles = topRightQ;  
      
      if(parentNode->isLeaf == 0){
        if(botLeftQ.size() > 0){
          // std::cout<<"Recursing into botLeft:"<<std::endl;
          helper(parentNode->children[0], botLeftQ, botLeftBmin, botLeftBmax, level+1);
        }
        else{
          parentNode->children[0]->isLeaf = true;
        }
        if(botRightQ.size() > 0){
          // std::cout<<"Recursing into botRight:"<<std::endl;
          helper(parentNode->children[1], botRightQ, botRightBmin, botRightBmax, level+1);
        }
        else{
          parentNode->children[1]->isLeaf = true;
        }
        if(topLeftQ.size() > 0){
          // std::cout<<"Recursing into topLeft:"<<std::endl;
          helper(parentNode->children[2], topLeftQ, topLeftBmin, topLeftBmax, level+1);
        }
        else{
          parentNode->children[2]->isLeaf = true;
        }
        if(topRightQ.size() > 0){
          // std::cout<<"Recursing into topRight:"<<std::endl;
          helper(parentNode->children[3], topRightQ, topRightBmin, topRightBmax, level+1);
        }
        else{
          parentNode->children[3]->isLeaf = true;
        }
      }
      
    std::cout<<"Level: "<<level<<" done"<<std::endl;
  }

  std::unique_ptr<QuadTreeNode> buildQuadTree(std::vector<Particle> &particles,
                                              Vec2 bmin, Vec2 bmax) {
    // TODO: implement a function that builds and returns a quadtree containing
    // particles.
    auto root = std::make_unique<QuadTreeNode>();
    root->particles.insert(root->particles.end(), particles.begin(), particles.end());
    auto level = 0;
    helper(root, root->particles, bmin, bmax, level);
    return root;
  }
  virtual std::unique_ptr<AccelerationStructure>
  buildAccelerationStructure(std::vector<Particle> &particles) {
    // build quad-tree
    auto quadTree = std::make_unique<QuadTree>();
    std::cout<<"In buildAccelerationStructure"<<std::endl;
    // find bounds
    Vec2 bmin(1e30f, 1e30f);
    Vec2 bmax(-1e30f, -1e30f);

    for (auto &p : particles) {
      bmin.x = fminf(bmin.x, p.position.x);
      bmin.y = fminf(bmin.y, p.position.y);
      bmax.x = fmaxf(bmax.x, p.position.x);
      bmax.y = fmaxf(bmax.y, p.position.y);
    }
    std::cout<<"particles count:"<<particles.size()<<std::endl;
    std::cout<<"Bbox X min: "<<bmin.x<<", max: "<<bmax.x<<std::endl;
    std::cout<<"Bbox Y min: "<<bmin.y<<", max: "<<bmax.y<<std::endl;
    quadTree->bmin = bmin;
    quadTree->bmax = bmax;

    // build nodes
    quadTree->root = buildQuadTree(particles, bmin, bmax);
    if (!quadTree->checkTree()) {
      std::cout << "Your Tree has Error!" << std::endl;
    }

    return quadTree;
  }

  Vec2 computeForceWithQuadTree(const Particle& body, std::vector<Particle> &particles, 
            std::unique_ptr<QuadTreeNode>& quadTreeNode, StepParameters& params){
    Vec2 force = Vec2(0.0f, 0.0f);
    float theta = 0.5;
    //Get the distance between centre of mass of this quadrant and the particle that you want to 
    float dist = (quadTreeNode->centreofmass - body.position).length();
    float side_dist_ratio = (quadTreeNode->bmax.x - quadTreeNode->bmin.x)/dist;
    std::cout<<"particle id: "<< body.id<<",particle x:"<<body.position.x<<", y:"<<body.position.y<<std::endl;
    std::cout<<"S/D:"<<side_dist_ratio<<std::endl;
    std::cout<<"centre of mass x="<<quadTreeNode->centreofmass.x<<",y="<<quadTreeNode->centreofmass.y<<std::endl;
    /*
    if(quadTreeNode->isLeaf){

    }
    else{
      if( side_dist_ratio > theta){

      }
      else{
          //Create a particle representing the sum of all masses
          force = quadTreeNode->totalMass;
      }
    }
    */
    return force;
  }

  virtual void simulateStep(AccelerationStructure *accel,
                            std::vector<Particle> &particles,
                            std::vector<Particle> &newParticles,
                            StepParameters params) override {
    // TODO: implement sequential version of quad-tree accelerated n-body
    // simulation here, using quadTree as acceleration structure
    QuadTree* quadTree = dynamic_cast<QuadTree *>(accel);
    float theta = 0.5;
    
    for (int i = 0; i < (int)particles.size(); i++) {
      auto body = particles[i];
      Vec2 force = Vec2(0.0f, 0.0f);
      //Calculate force on this particle due to other particles
      //Recursively compute force
      force += computeForceWithQuadTree(body, particles, quadTree->root, params);
    }
  }
};

std::unique_ptr<INBodySimulator> createSequentialNBodySimulator() {
  return std::make_unique<SequentialNBodySimulator>();
}
