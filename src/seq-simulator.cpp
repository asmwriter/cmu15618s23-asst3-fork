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
    parentNode->totalMass = 0; 
    //Check if the number of particles is more than QuadTreeLeafSize
    if(particles.size() <= QuadTreeLeafSize){
      //Then this is the leaf node.
      parentNode->isLeaf = 1;
      Vec2 weightedPos = 0.0;
      printf("leaf node");
      for(auto &particle: particles){
        parentNode->totalMass += particle.mass;
        weightedPos += particle.position * particle.mass;
      }
      parentNode->centreofmass = weightedPos * (1/parentNode->totalMass);
      std::cout<<"centre of mass - x:"<<parentNode->centreofmass.x<<", y:"<<parentNode->centreofmass.y<<std::endl;
      return;
    }
    else{
      parentNode->isLeaf = 0;
      
      //Insert children nodes
      Vec2 pivot = (bmin + bmax) * 0.5f;
      std::cout<<"pivot X: "<<pivot.x<<", Y: "<<pivot.y<<std::endl;
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
      Vec2 weightedPos = 0.0;
      for(auto &particle: particles){
        parentNode->totalMass += particle.mass;
        weightedPos += particle.position * particle.mass;
        std::cout<<"particle-x:"<<particle.position.x<<",y:"<<particle.position.y<<std::endl;
        if(particle.position.x >= topLeftBmin.x && particle.position.x <= topLeftBmax.x &&
             particle.position.y >= topLeftBmin.y && particle.position.y <= topLeftBmax.y){
              std::cout<<"top left"<<std::endl;
              topLeftQ.push_back(particle);
        }
        if(particle.position.x >= topRightBmin.x && particle.position.x <= topRightBmax.x &&
             particle.position.y >= topRightBmin.y && particle.position.y <= topRightBmax.y){
              std::cout<<"top right"<<std::endl;
              topRightQ.push_back(particle);
        }
        if(particle.position.x >= botLeftBmin.x && particle.position.x <= botLeftBmax.x &&
             particle.position.y >= botLeftBmin.y && particle.position.y <= botLeftBmax.y){
              std::cout<<"bottom left"<<std::endl;
              botLeftQ.push_back(particle);
        }
        if(particle.position.x >= botRightBmin.x && particle.position.x <= botRightBmax.x &&
             particle.position.y >= botRightBmin.y && particle.position.y <= botRightBmax.y){
              std::cout<<"bottom right"<<std::endl;
              botRightQ.push_back(particle);
        }
      }

      parentNode->centreofmass = weightedPos * (1/parentNode->totalMass);
      std::cout<<"centre of mass - x:"<<parentNode->centreofmass.x<<", y:"<<parentNode->centreofmass.y<<std::endl;
      parentNode->children[2] =  std::make_unique<QuadTreeNode>(); 
      parentNode->children[2]->particles = topLeftQ;
      std::cout<<"Recursing into topLeft:"<<std::endl;
      helper(parentNode->children[2], topLeftQ, topLeftBmin, topLeftBmax, level+1);
      
      parentNode->children[3] =  std::make_unique<QuadTreeNode>(); 
      parentNode->children[3]->particles = topRightQ;
      std::cout<<"Recursing into topRight:"<<std::endl;
      helper(parentNode->children[3], topRightQ, topRightBmin, topRightBmax, level+1);
     
      parentNode->children[0] =  std::make_unique<QuadTreeNode>(); 
      parentNode->children[0]->particles = botLeftQ;
      std::cout<<"Recursing into botLeft:"<<std::endl;
      helper(parentNode->children[0], botLeftQ, botLeftBmin, botLeftBmax, level+1);
    
      parentNode->children[1] =  std::make_unique<QuadTreeNode>(); 
      parentNode->children[1]->particles = botRightQ;
      std::cout<<"Recursing into botRight:"<<std::endl;
      helper(parentNode->children[1], botRightQ, botRightBmin, botRightBmax, level+1);
    }
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

    // find bounds
    Vec2 bmin(1e30f, 1e30f);
    Vec2 bmax(-1e30f, -1e30f);

    for (auto &p : particles) {
      bmin.x = fminf(bmin.x, p.position.x);
      bmin.y = fminf(bmin.y, p.position.y);
      bmax.x = fmaxf(bmax.x, p.position.x);
      bmax.y = fmaxf(bmax.y, p.position.y);
    }

    quadTree->bmin = bmin;
    quadTree->bmax = bmax;

    // build nodes
    quadTree->root = buildQuadTree(particles, bmin, bmax);
    if (!quadTree->checkTree()) {
      std::cout << "Your Tree has Error!" << std::endl;
    }

    return quadTree;
  }

  Vec2 computeForceWithQuadTree(const Particle& body, std::vector<Particle> &particles, std::unique_ptr<QuadTreeNode> quadTreeNode){
    Vec2 force = Vec2(0.0f, 0.0f);
    //Get the distance between centre of mass of this quadrant and the particle that you want to 
    float dist = (quadTreeNode->centreofmass - body.position).length();
    std::cout<<"particle id: "<< body.id<<std::endl;
    std::cout<<"centre of mass x="<<quadTreeNode->centreofmass.x<<",y="<<quadTreeNode->centreofmass.y<<std::endl;
    return force;
  }

  virtual void simulateStep(AccelerationStructure *accel,
                            std::vector<Particle> &particles,
                            std::vector<Particle> &newParticles,
                            StepParameters params) override {
    // TODO: implement sequential version of quad-tree accelerated n-body
    // simulation here, using quadTree as acceleration structure
    QuadTree* quadTree = dynamic_cast<QuadTree *>(buildAccelerationStructure(particles));
    float theta = 0.5;
    
    for (int i = 0; i < (int)particles.size(); i++) {
      auto body = particles[i];
      printf("particle = %d, id", body.id);
      Vec2 force = Vec2(0.0f, 0.0f);
      //Calculate force on this particle due to other particles
      //Recursively compute force
      force += computeForceWithQuadTree(body, particles, quadTree->root)
    }
  }
};

std::unique_ptr<INBodySimulator> createSequentialNBodySimulator() {
  return std::make_unique<SequentialNBodySimulator>();
}
