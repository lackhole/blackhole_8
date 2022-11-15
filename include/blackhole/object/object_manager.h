//
// Created by YongGyu Lee on 2022/11/15.
//

#ifndef BLACKHOLE_OBJECT_OBJECT_MANAGER_H_
#define BLACKHOLE_OBJECT_OBJECT_MANAGER_H_

#include <algorithm>
#include <functional>
#include <map>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include "blackhole/object/object.h"

namespace blackhole {

template<typename T>
class ObjectManager {
 public:
  using value_type = T;
  using object_type = DrawableObject<T>;
  using key_type = int;
  using mapped_type = std::unique_ptr<object_type>;
  using point_type = typename object_type::point_type;
  using vector_type = typename object_type::vector_type;

  static ObjectManager& GetInstance() {
    static ObjectManager* manager = new ObjectManager();
    return *manager;
  }

  std::pair<const key_type, object_type*> InsertObject(std::unique_ptr<object_type> object) {
    auto p = objects_.emplace(objects_.size(), std::move(object));
    return {p.first->first, p.first->second.get()};
  }

  template<typename U, std::enable_if_t<std::is_base_of_v<object_type, U>, int> = 0>
  std::pair<const key_type, U*> InsertObject(std::unique_ptr<U> object) {
    auto p = objects_.emplace(objects_.size(), std::move(object));
    return {p.first->first, static_cast<U*>(p.first->second.get())};
  }

  template<typename U, typename ...Args, std::enable_if_t<
    std::conjunction_v<
      std::is_base_of<object_type, U>,
      std::is_constructible<U, Args&&...>
    >, int> = 0>
  std::pair<const key_type, U*> InsertObject(Args&&... args) {
    return InsertObject(std::make_unique<U>(std::forward<Args>(args)...));
  }

//  auto FindObject(int key) {
//
//  }

  // Return a collided object
  const object_type*
  FindCollision(const point_type& p1, const point_type& p2, point_type* intersection) const {
    std::map<value_type, std::pair<const object_type*, point_type>> collisions; // distance^2, objects

    point_type inter;
    for (const auto& [_, object] : objects_) {
      if (object->Collide(p1, p2, &inter)) {
        const auto d = p1 - inter;
        collisions.emplace(d.dot(d), std::make_pair(object.get(), inter));
      }
    }

    if (collisions.empty())
      return nullptr;
    *intersection = collisions.begin()->second.second;
    return collisions.begin()->second.first;
  }

 private:
  ObjectManager() = default;
  ObjectManager(const ObjectManager&) = delete;
  ObjectManager& operator=(const ObjectManager&) = delete;

  std::unordered_map<key_type, mapped_type> objects_;
};

} // namespace blackhole

#endif // BLACKHOLE_OBJECT_OBJECT_MANAGER_H_
