#include "reactive_hokuyo_alg.h"

ReactiveHokuyoAlgorithm::ReactiveHokuyoAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

ReactiveHokuyoAlgorithm::~ReactiveHokuyoAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void ReactiveHokuyoAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=config;
  
  this->unlock();
}

// ReactiveHokuyoAlgorithm Public API
