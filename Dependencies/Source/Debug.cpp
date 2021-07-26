#include "Debug.h"
#include "Mesh.h"
#include "Camera.h"


static ImGuizmo::OPERATION mCurrentGizmoOperation(ImGuizmo::TRANSLATE);

// TODO modifica sa ia ca parametru glm::vec3 si sa nu mai faca decompose
void EditTransform(Camera* cam, glm::mat4* cameraView, glm::mat4* cameraProjection, Mesh* mesh)
{
    static ImGuizmo::MODE mCurrentGizmoMode(ImGuizmo::LOCAL);
    static bool useSnap = false;
    static float snap[3] = { 1.f, 1.f, 1.f };
    static float bounds[] = { -0.5f, -0.5f, -0.5f, 0.5f, 0.5f, 0.5f };
    static float boundsSnap[] = { 0.1f, 0.1f, 0.1f };
    static bool boundSizing = false;
    static bool boundSizingSnap = false;

    //if (editTransformDecomposition)
    //{
    if (ImGui::IsKeyPressed(90))
        mCurrentGizmoOperation = ImGuizmo::TRANSLATE;
    if (ImGui::IsKeyPressed(69))
        mCurrentGizmoOperation = ImGuizmo::ROTATE;
    if (ImGui::IsKeyPressed(82)) // r Key
        mCurrentGizmoOperation = ImGuizmo::SCALE;
    ///if (ImGui::RadioButton("Translate", mCurrentGizmoOperation == ImGuizmo::TRANSLATE))
    ///    mCurrentGizmoOperation = ImGuizmo::TRANSLATE;
    ///ImGui::SameLine();
    ///if (ImGui::RadioButton("Rotate", mCurrentGizmoOperation == ImGuizmo::ROTATE))
    ///    mCurrentGizmoOperation = ImGuizmo::ROTATE;
    ///ImGui::SameLine();
    ///if (ImGui::RadioButton("Scale", mCurrentGizmoOperation == ImGuizmo::SCALE))
    ///    mCurrentGizmoOperation = ImGuizmo::SCALE;
    float matrixTranslation[3], matrixRotation[3], matrixScale[3];

    matrixTranslation[0] = mesh->localTransform.tr[0];
    matrixTranslation[1] = mesh->localTransform.tr[1];
    matrixTranslation[2] = mesh->localTransform.tr[2];

    matrixRotation[0] = mesh->localTransform.rot[0];
    matrixRotation[1] = mesh->localTransform.rot[1];
    matrixRotation[2] = mesh->localTransform.rot[2];

    matrixScale[0] = mesh->localTransform.sc[0];
    matrixScale[1] = mesh->localTransform.sc[1];
    matrixScale[2] = mesh->localTransform.sc[2];

    //ImGuizmo::DecomposeMatrixToComponents(matrix, matrixTranslation, matrixRotation, matrixScale); // TODO scapa de asta

    ///ImGui::InputFloat3("Tr", matrixTranslation);
    ///ImGui::InputFloat3("Rt", matrixRotation);
    ///ImGui::InputFloat3("Sc", matrixScale);
    //ImGuizmo::RecomposeMatrixFromComponents(matrixTranslation, matrixRotation, matrixScale, matrix);

    ///if (mCurrentGizmoOperation != ImGuizmo::SCALE)
    ///{
    ///    if (ImGui::RadioButton("Local", mCurrentGizmoMode == ImGuizmo::LOCAL))
    ///        mCurrentGizmoMode = ImGuizmo::LOCAL;
    ///    ImGui::SameLine();
    ///    if (ImGui::RadioButton("World", mCurrentGizmoMode == ImGuizmo::WORLD))
    ///        mCurrentGizmoMode = ImGuizmo::WORLD;
    ///}
    ///if (ImGui::IsKeyPressed(83))
    ///    useSnap = !useSnap;
    ///ImGui::Checkbox("", &useSnap);
    ///ImGui::SameLine();

    ///switch (mCurrentGizmoOperation)
    ///{
    ///    case ImGuizmo::TRANSLATE:
    ///        ImGui::InputFloat3("Snap", &snap[0]);
    ///        break;
    ///    case ImGuizmo::ROTATE:
    ///        ImGui::InputFloat("Angle Snap", &snap[0]);
    ///        break;
    ///    case ImGuizmo::SCALE:
    ///        ImGui::InputFloat("Scale Snap", &snap[0]);
    ///        break;
    ///}
    ///ImGui::Checkbox("Bound Sizing", &boundSizing);
    ///if (boundSizing)
    ///{
    ///    ImGui::PushID(3);
    ///    ImGui::Checkbox("", &boundSizingSnap);
    ///    ImGui::SameLine();
    ///    ImGui::InputFloat3("Snap", boundsSnap);
    ///    ImGui::PopID();
    ///}
    ///}

    ImGuiIO& io = ImGui::GetIO();
    float viewManipulateRight = io.DisplaySize.x;
    float viewManipulateTop = 0;

    bool useWindow = false;

    ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);

    float *camview = (float*)glm::value_ptr(*cameraView);
    float *proj = (float*)glm::value_ptr(*cameraProjection);
    float* matrix = (float*)glm::value_ptr(mesh->getTransform());

    //ImGuizmo::DrawCubes(cameraView, cameraProjection, &objectMatrix[0][0], gizmoCount);
    ImGuizmo::Manipulate(camview, proj, mCurrentGizmoOperation, mCurrentGizmoMode, matrix, NULL, useSnap ? &snap[0] : NULL, boundSizing ? bounds : NULL, boundSizingSnap ? boundsSnap : NULL);
    //ImGuizmo::ViewManipulate(camview, glm::length(cam->position), ImVec2(viewManipulateRight - 128, viewManipulateTop), ImVec2(128, 128), 0x10101010);

    ImGuizmo::DecomposeMatrixToComponents(matrix, matrixTranslation, matrixRotation, matrixScale);

    mesh->localTransform.tr = glm::vec3(matrixTranslation[0], matrixTranslation[1], matrixTranslation[2]);
    //mesh->localTransform.rot = glm::vec3(mesh->rotation[0], mesh->rotation[1], mesh->rotation[2]);
    //mesh->localTransform.sc = glm::vec3(mesh->scale[0], mesh->scale[1], mesh->scale[2]);
}

std::string getTabs(int n) {
    std::string s = "";
    s.append(n, '\t');    
    return s;
}
