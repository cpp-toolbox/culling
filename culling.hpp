#ifndef CULLING_HPP
#define CULLING_HPP

#include <glm/glm.hpp>
#include <array>

#include "sbpt_generated_includes.hpp"

class ICuller {
  public:
    virtual ~ICuller() = default;
    virtual bool is_visible(const std::vector<glm::vec3> &xyz_positions, Transform &transform) = 0;
};

// NOTE: I also want to make a version of this or something that is designed ot work on orthographic top down type
// things as well
class FrustumCuller : public ICuller {
  public:
    explicit FrustumCuller(ICamera &cam, const unsigned int &screen_width_px, const unsigned int &screen_height_px)
        : camera(cam), screen_width_px(screen_width_px), screen_height_px(screen_height_px) {}

    template <draw_info::IVPLike IVPX> bool is_visible(IVPX &ivp_x) {
        return is_visible(ivp_x.xyz_positions, ivp_x.transform);
    }

    bool is_visible(const std::vector<glm::vec3> &xyz_positions, Transform &transform) override {
        auto frustum = camera.get_visible_frustum_world_space();
        auto local_aabb = vertex_geometry::AxisAlignedBoundingBox(xyz_positions);
        auto corners = get_aabb_corners_world(local_aabb, transform);
        return frustum.intersects_points(corners);
    }

    // NOTE: if we ever need to redo this then I want to redo it by using connect n-gon from vertex geom.
    // draw_info::IndexedVertexPositions generate_frustum_ivp(bool center_at_origin = false) {
    //     // Camera params
    //     float aspect = static_cast<float>(screen_width_px) / screen_height_px;
    //     float fov_y = glm::radians(camera.fov.get()); // vertical FOV
    //     float near_z = camera.near_plane;
    //     float far_z = camera.far_plane;
    //     far_z = 2;
    //
    //     // Half-sizes of near/far planes
    //     float tan_half_fov_y = tanf(fov_y * 0.5f);
    //     float near_height = 2.0f * near_z * tan_half_fov_y;
    //     float near_width = near_height * aspect;
    //     float far_height = 2.0f * far_z * tan_half_fov_y;
    //     float far_width = far_height * aspect;
    //
    //     // Define frustum corners in camera space
    //     std::array<glm::vec3, 8> cam_corners = {
    //         // Near plane
    //         glm::vec3(-near_width / 2, -near_height / 2, -near_z), glm::vec3(near_width / 2, -near_height / 2,
    //         -near_z), glm::vec3(-near_width / 2, near_height / 2, -near_z), glm::vec3(near_width / 2, near_height /
    //         2, -near_z),
    //         // Far plane
    //         glm::vec3(-far_width / 2, -far_height / 2, -far_z), glm::vec3(far_width / 2, -far_height / 2, -far_z),
    //         glm::vec3(-far_width / 2, far_height / 2, -far_z), glm::vec3(far_width / 2, far_height / 2, -far_z)};
    //
    //     std::vector<glm::vec3> world_corners;
    //     world_corners.reserve(8);
    //
    //     // --- Manually construct TRS matrix from Transform ---
    //     glm::vec3 t = camera.transform.get_translation();
    //     glm::vec3 s = camera.transform.get_scale();
    //     glm::vec3 r = camera.transform.get_rotation(); // in turns (0..1)
    //     glm::vec3 r_rad = r * glm::two_pi<float>();    // convert turns to radians
    //
    //     // Translation matrix
    //     glm::mat4 T = glm::translate(glm::mat4(1.0f), t);
    //
    //     // Rotation matrices (Euler order: pitch=X, yaw=Y, roll=Z)
    //     glm::mat4 Rx = glm::rotate(glm::mat4(1.0f), r_rad.x, glm::vec3(1, 0, 0));
    //     glm::mat4 Ry = glm::rotate(glm::mat4(1.0f), -r_rad.y - glm::two_pi<float>() / 4, glm::vec3(0, 1, 0));
    //     glm::mat4 Rz = glm::rotate(glm::mat4(1.0f), r_rad.z, glm::vec3(0, 0, 1));
    //     glm::mat4 R = Ry * Rx * Rz; // assuming YXZ order (common for FPS camera)
    //
    //     // Scale matrix
    //     glm::mat4 S = glm::scale(glm::mat4(1.0f), s);
    //
    //     // Combine to world matrix
    //     glm::mat4 world_matrix = T * R * S; // TRS order
    //
    //     for (auto &c : cam_corners) {
    //         glm::vec4 world = world_matrix * glm::vec4(c, 1.0f);
    //         world_corners.push_back(glm::vec3(world));
    //     }
    //
    //     if (center_at_origin) {
    //         for (auto &p : world_corners) {
    //             p -= t; // shift apex to origin
    //         }
    //     }
    //
    //     // Triangle indices
    //     std::vector<unsigned int> indices = {
    //         0, 1, 2, 1, 3, 2, // near
    //         4, 6, 5, 5, 6, 7, // far
    //         0, 2, 4, 2, 6, 4, // left
    //         1, 5, 3, 3, 5, 7, // right
    //         2, 3, 6, 3, 7, 6, // top
    //         0, 4, 1, 1, 4, 5  // bottom
    //     };
    //
    //     return draw_info::IndexedVertexPositions(indices, world_corners);
    // }

  private:
    ICamera &camera;
    const unsigned int &screen_width_px, &screen_height_px;
    std::array<glm::vec4, 6> planes; // frustum planes

    std::array<glm::vec3, 8> get_aabb_corners_world(const vertex_geometry::AxisAlignedBoundingBox &box,
                                                    Transform &transform) const {
        glm::mat4 model = transform.get_transform_matrix();
        std::array<glm::vec3, 8> corners = box.get_corners();
        for (auto &c : corners) {
            glm::vec4 world = model * glm::vec4(c, 1.0f);
            c = glm::vec3(world);
        }
        return corners;
    }
};

#endif // CULLING_HPP
