//
// Created by felixfifi on 17.05.20.
//

#include "SceneLoader.h"

#define TINYOBJLOADER_IMPLEMENTATION

#include "tiny_obj_loader.h"

#include <glm/gtc/random.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <filesystem>

#define STB_IMAGE_IMPLEMENTATION

#include <stb_image.h>

#include "tinyxml2.h"

#include "json.hpp"
#include "WeightedSampler.h"
#include "MitsubaXML.h"

using json = nlohmann::json;
using namespace tinyxml2;

SceneLoader::SceneLoader(const std::string &filepath, const std::string &objectBaseDir,
                         const std::string &materialBaseDir, const std::string &textureBaseDir,
                         std::shared_ptr<VulkanOps> vulkanOps, vk::PhysicalDevice &physicalDevice,
                         uint32_t graphicsQueueIndex)
        : objectBaseDir(objectBaseDir), materialBaseDir(materialBaseDir), textureBaseDir(textureBaseDir),
          vulkanOps(vulkanOps),
          device(vulkanOps->getDevice()) {

    std::filesystem::path path = filepath;

    if (path.extension().string() == ".xml") {
        parseMitsubaSceneFile(filepath);
    } else if (path.extension().string() == ".json") {
        parseSceneFile(filepath);
    }

    createVulkanObjects(physicalDevice, graphicsQueueIndex);
}

void SceneLoader::createVulkanObjects(vk::PhysicalDevice &physicalDevice, uint32_t graphicsQueueIndex) {
    rtBuilder.setup(device, physicalDevice, graphicsQueueIndex);
    createMaterialBuffer();
    createInstanceInfoBuffer();
    createLightsBuffers();
    createBottomLevelAS();
    createTopLevelAS();

    // To prevent empty descriptor sets
    if (materials.empty()) {
        materials.push_back({});
    }
    if (textures.empty()) {
        Texture texture;

        std::vector<char> pixels{
                0, 0, 0, 0
        };

        vulkanOps->createImageFromData(pixels, 1, 1, vk::Format::eR8G8B8A8Srgb, texture.image, texture.imageMemory,
                                       texture.imageView);

        vk::SamplerCreateInfo samplerCreateInfo{{},
                                                vk::Filter::eLinear,
                                                vk::Filter::eLinear,
                                                vk::SamplerMipmapMode::eNearest,
                                                vk::SamplerAddressMode::eRepeat,
                                                vk::SamplerAddressMode::eRepeat,
                                                vk::SamplerAddressMode::eRepeat,
                                                {},
                                                false,
                                                0};

        texture.sampler = device.createSampler(samplerCreateInfo);

        textures.push_back(texture);

    }
}

void SceneLoader::loadModel(const std::string &objFilePath) {
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> tinyMaterials;
    readObjFile(objFilePath, attrib, shapes, tinyMaterials);

    int materialIndexOffset = materials.size();

    addMaterials(tinyMaterials);

    addModel(attrib, shapes, materialIndexOffset);

}

void SceneLoader::readObjFile(const std::string &objFilePath, tinyobj::attrib_t &attrib,
                              std::vector<tinyobj::shape_t> &shapes,
                              std::vector<tinyobj::material_t> &tinyMaterials) const {
    std::string warn, err;

    if (!tinyobj::LoadObj(&attrib, &shapes, &tinyMaterials, &warn, &err, objFilePath.c_str(),
                          materialBaseDir.c_str())) {
        throw std::runtime_error(warn + err);
    }
}

void SceneLoader::addMaterials(const std::vector<tinyobj::material_t> &tinyMaterials) {
    for (const auto &tinyMaterial : tinyMaterials) {
        Material material{};

        material.lightColor = {tinyMaterial.emission[0], tinyMaterial.emission[1], tinyMaterial.emission[2]};
        material.diffuse = {tinyMaterial.diffuse[0], tinyMaterial.diffuse[1], tinyMaterial.diffuse[2]};
        material.specular = {tinyMaterial.specular[0], tinyMaterial.specular[1], tinyMaterial.specular[2]};
        material.specularHighlight = tinyMaterial.shininess;
        material.refractionIndex = tinyMaterial.ior;
        material.refractionIndexInv = 1.0f / tinyMaterial.ior;

        switch (tinyMaterial.illum) {
            case 0:
            case 1:
                material.type = eDiffuse;
                break;
            case 2:
                material.type = ePhong;
                break;
            case 3:
                material.type = eSpecular;
                break;
            case 4:
            case 7:
                material.type = eDielectric;
                break;
            case 11:
                material.type = eLight;
                break;
            default:
                throw std::runtime_error("Unknown illum mode");
        }

        if (!tinyMaterial.diffuse_texname.empty()) {
            material.textureIdDiffuse = addTexture(tinyMaterial.diffuse_texname);
        }

        if (!tinyMaterial.specular_texname.empty()) {
            material.textureIdDiffuse = addTexture(tinyMaterial.specular_texname);
        }

        materials.push_back(material);
    }
}

int SceneLoader::addTexture(const std::string &textureName) {
    std::filesystem::path path = textureBaseDir;
    path /= textureName;


    // Return existing texture id if it was already loaded
    if (pathTextureIdMapping.contains(path.string())) {
        return pathTextureIdMapping[path.string()];
    }

    if (path.string().find('\\') != -1)
        path = path.string().replace(path.string().find('\\'), 1, "/");
    path = path.make_preferred().lexically_normal();

    int width, height, channels;
    const int outputChannel = 4;
    stbi_uc *data = stbi_load(path.c_str(), &width, &height, &channels, outputChannel);

    if (!data) {
        throw std::runtime_error("Could not load texture file " + path.string());
    }

    std::vector<stbi_uc> dataVector(data, data + sizeof(data[0]) * width * height * outputChannel);
    free(data);

    vk::Format format = vk::Format::eR8G8B8A8Srgb;

    Texture texture{};

    vulkanOps->createImageFromData(dataVector, width, height, format, texture.image, texture.imageMemory,
                                   texture.imageView);

    vk::SamplerCreateInfo samplerCreateInfo{{},
                                            vk::Filter::eLinear,
                                            vk::Filter::eLinear,
                                            vk::SamplerMipmapMode::eNearest,
                                            vk::SamplerAddressMode::eRepeat,
                                            vk::SamplerAddressMode::eRepeat,
                                            vk::SamplerAddressMode::eRepeat,
                                            {},
                                            false,
                                            0};

    texture.sampler = device.createSampler(samplerCreateInfo);


    textures.push_back(texture);

    int textureIndex = textures.size() - 1;

    pathTextureIdMapping[path.string()] = textureIndex;
    return textureIndex;
}

void SceneLoader::addModel(const tinyobj::attrib_t &attrib, const std::vector<tinyobj::shape_t> &shapes,
                           int materialIndexOffset) {
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    std::vector<int> emissiveFaces;
    converteObjData(shapes, attrib, materialIndexOffset, -1, vertices, indices, emissiveFaces);


    models.emplace_back(vertices, indices, vulkanOps);
    emissiveFacesPerModel.push_back(emissiveFaces);
}

void SceneLoader::converteObjData(const std::vector<tinyobj::shape_t> &shapes, const tinyobj::attrib_t &attrib,
                                  int materialIndexOffset, int materialIndexOverride, std::vector<Vertex> &outVertices,
                                  std::vector<uint32_t> &outIndices, std::vector<int> &outEmissiveFaces) const {
    bool hasNormals = !attrib.normals.empty();
    bool hasTexCoords = !attrib.texcoords.empty();

    std::unordered_map<Vertex, uint32_t> uniqueVertices = {};
    for (const auto &shape : shapes) { // TODO: separate obj by shape instead of joing all
        unsigned long numFaces = shape.mesh.indices.size() / VERTICES_PER_FACE;
        for (int iFace = 0; iFace < numFaces; ++iFace) {
            int materialIndex = materialIndexOverride;

            if (materialIndexOverride < 0) {
                // Material offsets are per file, but we accumulate materials over multiple files
                materialIndex = materialIndexOffset + shape.mesh.material_ids[iFace];
            }

            std::vector<Vertex> faceVertices;

            for (int i = 0; i < VERTICES_PER_FACE; i++) {
                const auto &index = shape.mesh.indices[VERTICES_PER_FACE * iFace + i];
                Vertex vertex = {};

                vertex.pos = {
                        attrib.vertices[3 * index.vertex_index + 0],
                        attrib.vertices[3 * index.vertex_index + 1],
                        attrib.vertices[3 * index.vertex_index + 2]
                };

                if (hasNormals) {
                    vertex.normal = {
                            attrib.normals[3 * index.normal_index + 0],
                            attrib.normals[3 * index.normal_index + 1],
                            attrib.normals[3 * index.normal_index + 2]
                    };
                }

                if (hasTexCoords) {
                    vertex.texCoord = {
                            attrib.texcoords[2 * index.texcoord_index + 0],
                            1.0f - attrib.texcoords[2 * index.texcoord_index + 1]
                    };
                }

                vertex.materialIndex = materialIndex;

                faceVertices.push_back(vertex);
            }

            if (!hasNormals) {
                // Calculate normals per face
                glm::vec3 ab = faceVertices[1].pos - faceVertices[0].pos;
                glm::vec3 ac = faceVertices[2].pos - faceVertices[0].pos;
                glm::vec3 normal = glm::normalize(glm::cross(ab, ac));

                for (Vertex &v: faceVertices) {
                    v.normal = normal;
                }
            }

            for (Vertex &vertex: faceVertices) {
                // Check if there exist copies of this vertex with the same exact values
                if (uniqueVertices.count(vertex) == 0) {
                    uniqueVertices[vertex] = static_cast<uint32_t>(outVertices.size());
                    outVertices.push_back(vertex);
                }

                outIndices.push_back(uniqueVertices[vertex]);
            }


            if (materials[materialIndex].type == eLight) {
                outEmissiveFaces.push_back((outIndices.size() - 1) / VERTICES_PER_FACE);
            }

        }
    }
}

void SceneLoader::parseMitsubaSceneFile(const std::string &filepath) {
    XMLDocument document;
    document.LoadFile(filepath.c_str());

    if (document.Error()) {
        throw std::runtime_error(document.ErrorName());
    }

    XMLElement *xScene = document.FirstChildElement("scene");

    parseCameraSettings(xScene);
    std::map<std::string, int> definedMaterials = parseXmlBSDFs(xScene);

    XMLElement *xShape = xScene->FirstChildElement("shape");
    while (xShape) {
        std::string type = xShape->Attribute("type");

        // First get material
        int matIndex = -1;

        // Check if a bsdf definition is given in the shape itself
        XMLElement *xBSDF = xShape->FirstChildElement("bsdf");
        if (xBSDF) {
            std::string tmp;
            Material mat = parseXmlBSDF(xBSDF, tmp);

            matIndex = materials.size();
            materials.push_back(mat);
        }

        if (matIndex < 0) {
            // Else, check for refs
            XMLElement *xRef = xShape->FirstChildElement("ref");
            if (xRef && xRef->Attribute("name", "bsdf")) {
                std::string id = xRef->Attribute("id");

                matIndex = definedMaterials[id];
            }
        }

        if (matIndex < 0) {
            throw std::runtime_error("Material index not set");
        }

        // Check if emitter data is also set and create a new material to represent this
        XMLElement *xEmitter = xShape->FirstChildElement("emitter");
        if (xEmitter) {
            // As the BSDF definitions in Mitsuba files don't contain emittance, we now have to create a new material,
            // as multiple objects can use the same BSDF but different emittance values

            glm::vec3 radiance = getChildRGB(xEmitter, "radiance");

            // Not a reference, but a copy
            Material mat = materials[matIndex];

            mat.lightColor = radiance;
            mat.type = eLight;

            materials.push_back(mat);
            matIndex = materials.size() - 1;
        }

        if (type == "obj") {
            std::string filename = getChildString(xShape, "filename");
            filename = toObjPath(filename);

            tinyobj::attrib_t attrib;
            std::vector<tinyobj::shape_t> shapes;
            std::vector<tinyobj::material_t> tinyMaterials;
            readObjFile(filename, attrib, shapes, tinyMaterials);


            std::vector<Vertex> vertices;
            std::vector<uint32_t> indices;
            std::vector<int> emissiveFaces;
            converteObjData(shapes, attrib, -1, matIndex, vertices, indices, emissiveFaces);

            models.emplace_back(vertices, indices, vulkanOps);
            emissiveFacesPerModel.push_back(emissiveFaces);

            int modelIndex = models.size() - 1;
            int instanceIndex = instances.size();

            // A model definition is automatically an instance
            Instance instance{};
            instance.transform = glm::mat4(1);
            instance.normalTransform = glm::mat4(1);
            instance.iModel = modelIndex;

            if (!emissiveFaces.empty()) {
                Light light{};
                light.isPointLight = 0;
                light.instanceIndex = instanceIndex;

                lights.push_back(light);

                instance.iLight = lights.size() - 1;
            }

            instances.push_back(instance);
        } else if (type == "sphere") {
            // TODO: For now, represent spheres by loading the sphere obj and scaling it
            const float SPHERE_OBJ_RADIUS = 1.0f;
            const std::string SPHERE_OBJ_PATH = "unit_sphere.obj";

            std::string filename = SPHERE_OBJ_PATH;
            filename = toObjPath(filename);

            tinyobj::attrib_t attrib;
            std::vector<tinyobj::shape_t> shapes;
            std::vector<tinyobj::material_t> tinyMaterials;
            readObjFile(filename, attrib, shapes, tinyMaterials);

            std::vector<Vertex> vertices;
            std::vector<uint32_t> indices;
            std::vector<int> emissiveFaces;
            converteObjData(shapes, attrib, -1, matIndex, vertices, indices, emissiveFaces);

            models.emplace_back(vertices, indices, vulkanOps);
            emissiveFacesPerModel.push_back(emissiveFaces);

            int modelIndex = models.size() - 1;
            int instanceIndex = instances.size();


            float radius = getChildFloat(xShape, "radius");
            glm::vec3 center = getChildPoint(xShape, "center");

            // A model definition is automatically an instance
            Instance instance{};
            float scale = radius / SPHERE_OBJ_RADIUS;
            glm::mat4 transform = glm::translate(glm::mat4(1), center);
            transform = glm::scale(transform, {scale, scale, scale});
            instance.transform = transform;

            glm::mat4 normalTransform = glm::scale(glm::mat4(1), {1.0f / scale, 1.0f / scale, 1.0f / scale});
            instance.normalTransform = normalTransform;
            instance.iModel = modelIndex;

            if (!emissiveFaces.empty()) {
                Light light{};
                light.isPointLight = 0;
                light.instanceIndex = instanceIndex;

                lights.push_back(light);

                instance.iLight = lights.size() - 1;
            }

            instances.push_back(instance);

        }

        xShape = xShape->NextSiblingElement("shape");
    }
}

std::map<std::string, int> SceneLoader::parseXmlBSDFs(XMLElement *xScene) {
    std::map<std::string, int> definedMaterials;

    XMLElement *xBSDF = xScene->FirstChildElement("bsdf");
    while (xBSDF) {
        std::string id;
        Material mat = parseXmlBSDF(xBSDF, id);

        if (definedMaterials.contains(id)) {
            throw std::runtime_error("Duplicate BSDF id");
        }

        definedMaterials[id] = materials.size();
        materials.push_back(mat);

        xBSDF = xBSDF->NextSiblingElement("bsdf");
    }

    return definedMaterials;
}

Material SceneLoader::parseXmlBSDF(XMLElement *xBSDF, std::string &outId) const {

    const char *id = xBSDF->Attribute("id");
    if (id) {
        outId = id;
    }
    std::string type = xBSDF->Attribute("type");

    Material mat;
    if (type == "phong") {
        mat.type = ePhong;
        mat.specular = getChildRGB(xBSDF, "specularReflectance");
        mat.diffuse = getChildRGB(xBSDF, "diffuseReflectance");
        mat.specularHighlight = getChildFloat(xBSDF, "exponent");
    } else if (type == "diffuse") {
        mat.type = eDiffuse;
        mat.specular = {0, 0, 0};
        mat.diffuse = getChildRGB(xBSDF, "reflectance");
        mat.specularHighlight = 0;
    } else if (type == "dielectric") {
        mat.type = eDielectric;
        mat.specular = {1, 1, 1};
        mat.refractionIndex = getChildFloat(xBSDF, "intIOR") / getChildFloat(xBSDF, "extIOR");
        mat.refractionIndexInv = 1.0f / mat.refractionIndex;
    } else {
        std::cerr << "Encountered unknown material type: " << type << std::endl;
    }

    return mat;
}

void SceneLoader::parseCameraSettings(XMLElement *xScene) {
    XMLElement *xSensor = xScene->FirstChildElement("sensor");

    if (xSensor) {
        XMLElement *xTransform = xSensor->FirstChildElement("transform");

        if (xTransform) {
            XMLElement *xLookAt = xTransform->FirstChildElement("lookAt");
            if (!xLookAt) {
                xLookAt = xTransform->FirstChildElement("lookat");
            }

            // LookAt element with origin target upDir
            if (xLookAt) {
                std::string tOrigin = xLookAt->Attribute("origin");
                std::string tTarget = xLookAt->Attribute("target");
                std::string tUpDir = xLookAt->Attribute("up");

                origin = parseCommaSpaceSeparatedVec3(tOrigin);
                target = parseCommaSpaceSeparatedVec3(tTarget);
                upDir = parseCommaSpaceSeparatedVec3(tUpDir);
            }

            // FOV
            XMLElement *xFov = xSensor->FirstChildElement("float");
            while (xFov && xFov->FindAttribute("name") && strcmp(xFov->FindAttribute("name")->Value(), "fov") != 0) {
                xFov = xFov->NextSiblingElement("float");
            }

            if (xFov) {
                vfov = xFov->FloatAttribute("value");
            }

        }
    }
}

void SceneLoader::parseSceneFile(const std::string &filepath) {// read a JSON file
    std::ifstream inFile(filepath);
    json j;
    inFile >> j;

    std::map<std::string, int> nameIndexMapping;

    parseModels(j, nameIndexMapping);

    parseInstances(j, nameIndexMapping);

    parsePointLights(j);

}

/**
 * Has to come after instances, because the area lights have to be added first
 * @param j
 */
void SceneLoader::parsePointLights(const json &j) {
    if (j.contains("lights")) {
        std::vector<json> jLights = j["lights"];

        for (auto jLight : jLights) {
            Light light{};
            light.isPointLight = true;
            std::vector<float> color = jLight["color"];
            light.color = {color[0], color[1], color[2]};

            std::vector<float> pos = jLight["position"];
            light.pos = {pos[0], pos[1], pos[2]};

            lights.push_back(light);
        }
    }
}

void SceneLoader::parseInstances(const json &j, std::map<std::string, int> &nameIndexMapping) {
    std::vector<json> jInstances = j["instances"];

    for (auto jInstance : jInstances) {
        std::string name = jInstance.items().begin().key();

        Instance instance{};
        instance.iModel = nameIndexMapping[name];

        // if any of the model faces are emissive -> add light struct
        if (!emissiveFacesPerModel[instance.iModel].empty()) {
            Light light{};
            light.isPointLight = false;
            light.instanceIndex = instances.size();
            lights.push_back(light);

            instance.iLight = lights.size() - 1;
        }

        glm::mat4 transform(1.0f);
        // Normals have to be transformed differently
        // Scale needs to be inverted
        glm::mat4 normalTransform(1.0f);

        std::unordered_map<std::string, json> properties = jInstance[name];


        if (properties.contains("translate")) {
            std::vector<float> translate = properties["translate"];
            transform = glm::translate(transform, {translate[0], translate[1], translate[2]});
        }

        if (properties.contains("rotate")) {
            std::vector<float> rotate = properties["rotate"];
            transform = glm::rotate(transform, glm::radians(rotate[0]), {1, 0, 0});
            transform = glm::rotate(transform, glm::radians(rotate[1]), {0, 1, 0});
            transform = glm::rotate(transform, glm::radians(rotate[2]), {0, 0, 1});

            normalTransform = glm::rotate(normalTransform, glm::radians(rotate[0]), {1, 0, 0});
            normalTransform = glm::rotate(normalTransform, glm::radians(rotate[1]), {0, 1, 0});
            normalTransform = glm::rotate(normalTransform, glm::radians(rotate[2]), {0, 0, 1});
        }

        if (properties.contains("scale")) {
            std::vector<float> scale = properties["scale"];
            transform = glm::scale(transform, {scale[0], scale[1], scale[2]});
            normalTransform = glm::scale(normalTransform, {1.0f / scale[0], 1.0f / scale[1], 1.0f / scale[2]});
        }

        instance.transform = transform;
        instance.normalTransform = normalTransform;

        instances.push_back(instance);
    }
}

void SceneLoader::parseModels(json &j, std::map<std::string, int> &nameIndexMapping) {
    std::vector<json> jModels = j["models"];

    for (auto jModel : jModels) {
        std::string name = jModel.items().begin().key();
        auto path = jModel[name].get<std::string>();

        loadModel(toObjPath(path));
        nameIndexMapping[name] = models.size() - 1;
    }
}

std::string SceneLoader::toObjPath(const std::string &path) {
    std::string res = (std::filesystem::path(objectBaseDir) / path);
    return res;
}

void SceneLoader::createMaterialBuffer() {
    vk::BufferUsageFlags usage = vk::BufferUsageFlagBits::eStorageBuffer;
    vulkanOps->createBufferFromData(materials, usage,
                                    vk::MemoryPropertyFlagBits::eDeviceLocal, materialBuffer, materialBufferMemory);
}

void SceneLoader::createInstanceInfoBuffer() {
    vk::BufferUsageFlags usage = vk::BufferUsageFlagBits::eStorageBuffer;
    vulkanOps->createBufferFromData(instances, usage,
                                    vk::MemoryPropertyFlagBits::eDeviceLocal, instanceInfoBuffer,
                                    instanceInfoBufferMemory);
}

void SceneLoader::createLightsBuffers() {
    createLightSamplersBuffer();

    vk::BufferUsageFlags usage = vk::BufferUsageFlagBits::eStorageBuffer;

    // Light count is padded with 0s to satisfy struct alignment
    std::vector<int> lightCount{static_cast<int>(lights.size()), 0, 0, 0};
    vulkanOps->createBufferFrom2Data(lightCount, lights, usage,
                                     vk::MemoryPropertyFlagBits::eDeviceLocal, lightsBuffer, lightsBufferMemory);
}

/**
 * Creates buffer that hold samplers to choose a random light, and if it is an area light, to choose a random face.
 * These random numbers should be weighted by the total power and face area.
 */
void SceneLoader::createLightSamplersBuffer() {
    std::vector<int> randomLightIndex = getLightSamplingVector();

    std::vector<FaceSample> randomTriIndicesPerLight = getFaceSamplingVector();

    vk::BufferUsageFlags usage = vk::BufferUsageFlagBits::eStorageBuffer;

    vulkanOps->createBufferFrom2Data(randomLightIndex, randomTriIndicesPerLight, usage,
                                     vk::MemoryPropertyFlagBits::eDeviceLocal, lightsSamplersBuffer,
                                     lightsSamplerBufferMemory);
}

std::vector<FaceSample> SceneLoader::getFaceSamplingVector() {
    std::vector<FaceSample> randomTriIndicesPerLight;
    for (auto &light : lights) {
        // Area lights are before all point lights
        if (light.isPointLight) {
            break;
        }

        Instance &instance = instances[light.instanceIndex];
        int iModel = instance.iModel;
        Model model = models[iModel];
        std::vector<float> areas(emissiveFacesPerModel[iModel].size());

        for (int iEmissiveFace = 0; iEmissiveFace < emissiveFacesPerModel[iModel].size(); ++iEmissiveFace) {
            areas[iEmissiveFace] = model.getFaceArea(emissiveFacesPerModel[iModel][iEmissiveFace], instance.transform);
        }

        WeightedSampler faceSampler(areas);

        // Set this area for the light for prob calculations
        // TODO: move
        light.area = faceSampler.getTotal();

        std::vector<float> probSampleFace = faceSampler.getProbabilities();

        std::vector<FaceSample> randomTriIndices(SIZE_TRI_RANDOM);
        for (int i = 0; i < SIZE_TRI_RANDOM; ++i) {
            int sample = faceSampler.sample();

            FaceSample faceSample{};
            faceSample.index = emissiveFacesPerModel[iModel][sample];
            faceSample.sampleProb = probSampleFace[sample];
            faceSample.faceArea = areas[sample];
            randomTriIndices[i] = faceSample;
        }

        randomTriIndicesPerLight.insert(randomTriIndicesPerLight.end(), randomTriIndices.begin(),
                                        randomTriIndices.end());
    }
    return randomTriIndicesPerLight;
}

std::vector<int> SceneLoader::getLightSamplingVector() {
    std::vector<float> powers;
    powers.reserve(lights.size());
    for (const auto &light : lights) {
        powers.push_back(1.0f); // TODO: Calculate per light power for weighting
    }

    WeightedSampler lightSampler(powers);

    // Set probabilities to sample a light
    std::vector<float> probSampleLight = lightSampler.getProbabilities();

    for (int iLight = 0; iLight < lights.size(); ++iLight) {
        lights[iLight].sampleProb = probSampleLight[iLight];
    }

    std::vector<int> randomLightIndex(SIZE_LIGHT_RANDOM);
    for (int i = 0; i < SIZE_LIGHT_RANDOM; ++i) {
        randomLightIndex[i] = lightSampler.sample();
    }
    return randomLightIndex;
}

std::array<vk::DescriptorSetLayoutBinding, BINDINGS_COUNT> SceneLoader::getDescriptorSetLayouts() {
    vk::DescriptorSetLayoutBinding vertexBufferBinding(1, vk::DescriptorType::eStorageBuffer, models.size(),
                                                       vk::ShaderStageFlagBits::eClosestHitKHR |
                                                       vk::ShaderStageFlagBits::eAnyHitKHR |
                                                       vk::ShaderStageFlagBits::eRaygenKHR);
    vk::DescriptorSetLayoutBinding indexBufferBinding(2, vk::DescriptorType::eStorageBuffer, models.size(),
                                                      vk::ShaderStageFlagBits::eClosestHitKHR |
                                                      vk::ShaderStageFlagBits::eAnyHitKHR |
                                                      vk::ShaderStageFlagBits::eRaygenKHR);
    vk::DescriptorSetLayoutBinding materialBufferBinding(3, vk::DescriptorType::eStorageBuffer, 1,
                                                         vk::ShaderStageFlagBits::eRaygenKHR |
                                                         vk::ShaderStageFlagBits::eAnyHitKHR);
    vk::DescriptorSetLayoutBinding instanceInfoBufferBinding(4, vk::DescriptorType::eStorageBuffer, 1,
                                                             vk::ShaderStageFlagBits::eClosestHitKHR |
                                                             vk::ShaderStageFlagBits::eAnyHitKHR |
                                                             vk::ShaderStageFlagBits::eRaygenKHR);
    vk::DescriptorSetLayoutBinding lightsBufferBinding(5, vk::DescriptorType::eStorageBuffer, 1,
                                                       vk::ShaderStageFlagBits::eRaygenKHR);
    vk::DescriptorSetLayoutBinding lightSamplersBufferBinding(6, vk::DescriptorType::eStorageBuffer, 1,
                                                              vk::ShaderStageFlagBits::eRaygenKHR);
    vk::DescriptorSetLayoutBinding texturesBinding(7, vk::DescriptorType::eCombinedImageSampler, textures.size(),
                                                   vk::ShaderStageFlagBits::eRaygenKHR |
                                                   vk::ShaderStageFlagBits::eAnyHitKHR, nullptr);

    return {vertexBufferBinding, indexBufferBinding, materialBufferBinding, instanceInfoBufferBinding,
            lightsBufferBinding, lightSamplersBufferBinding, texturesBinding};
}

std::array<vk::DescriptorPoolSize, BINDINGS_COUNT> SceneLoader::getDescriptorPoolSizes() {
    return {
            vk::DescriptorPoolSize(vk::DescriptorType::eStorageBuffer, models.size()),
            vk::DescriptorPoolSize(vk::DescriptorType::eStorageBuffer, models.size()),
            vk::DescriptorPoolSize(vk::DescriptorType::eStorageBuffer, 1),
            vk::DescriptorPoolSize(vk::DescriptorType::eStorageBuffer, 1),
            vk::DescriptorPoolSize(vk::DescriptorType::eStorageBuffer, 1),
            vk::DescriptorPoolSize(vk::DescriptorType::eStorageBuffer, 1),
            vk::DescriptorPoolSize(vk::DescriptorType::eCombinedImageSampler, textures.size())
    };
}

/**
 *
 * @param descriptorSet
 * @param outVertexBufferInfos Necessary as their memory location is used in the write descriptor sets
 * @param outIndexBufferInfos Necessary as their memory location is used in the write descriptor sets
 * @param outMaterialBufferInfo Necessary as their memory location is used in the write descriptor sets
 * @return
 */
std::array<vk::WriteDescriptorSet, BINDINGS_COUNT>
SceneLoader::getWriteDescriptorSets(const vk::DescriptorSet &descriptorSet,
                                    std::vector<vk::DescriptorBufferInfo> &outVertexBufferInfos,
                                    std::vector<vk::DescriptorBufferInfo> &outIndexBufferInfos,
                                    vk::DescriptorBufferInfo &outMaterialBufferInfo,
                                    vk::DescriptorBufferInfo &outInstanceInfoBufferInfo,
                                    vk::DescriptorBufferInfo &outLightsBufferInfo,
                                    vk::DescriptorBufferInfo &outLightSamplersBufferInfo,
                                    std::vector<vk::DescriptorImageInfo> &outTexturesInfos) {
    unsigned long modelCount = models.size();

    outVertexBufferInfos.clear();
    outIndexBufferInfos.clear();
    outTexturesInfos.clear();
    outVertexBufferInfos.reserve(modelCount);
    outIndexBufferInfos.reserve(modelCount);
    outTexturesInfos.reserve(textures.size());

    for (const auto &model: models) {
        vk::DescriptorBufferInfo vertexBufferInfo(model.vertexBuffer, 0, VK_WHOLE_SIZE);
        vk::DescriptorBufferInfo indexBufferInfo(model.indexBuffer, 0, VK_WHOLE_SIZE);

        outVertexBufferInfos.push_back(vertexBufferInfo);
        outIndexBufferInfos.push_back(indexBufferInfo);
    }

    for (const auto &texture: textures) {
        vk::DescriptorImageInfo textureInfo{texture.sampler, texture.imageView,
                                            vk::ImageLayout::eShaderReadOnlyOptimal};
        outTexturesInfos.push_back(textureInfo);
    }

    outMaterialBufferInfo = vk::DescriptorBufferInfo(materialBuffer, 0, VK_WHOLE_SIZE);
    outInstanceInfoBufferInfo = vk::DescriptorBufferInfo(instanceInfoBuffer, 0, VK_WHOLE_SIZE);
    outLightsBufferInfo = vk::DescriptorBufferInfo(lightsBuffer, 0, VK_WHOLE_SIZE);
    outLightSamplersBufferInfo = vk::DescriptorBufferInfo(lightsSamplersBuffer, 0, VK_WHOLE_SIZE);

    return {
            vk::WriteDescriptorSet(descriptorSet, 1, 0, modelCount,
                                   vk::DescriptorType::eStorageBuffer, nullptr,
                                   outVertexBufferInfos.data(), nullptr),
            vk::WriteDescriptorSet(descriptorSet, 2, 0, modelCount,
                                   vk::DescriptorType::eStorageBuffer, nullptr,
                                   outIndexBufferInfos.data(), nullptr),
            vk::WriteDescriptorSet(descriptorSet, 3, 0, 1,
                                   vk::DescriptorType::eStorageBuffer, nullptr,
                                   &outMaterialBufferInfo, nullptr),
            vk::WriteDescriptorSet(descriptorSet, 4, 0, 1,
                                   vk::DescriptorType::eStorageBuffer, nullptr,
                                   &outInstanceInfoBufferInfo, nullptr),
            vk::WriteDescriptorSet(descriptorSet, 5, 0, 1,
                                   vk::DescriptorType::eStorageBuffer, nullptr,
                                   &outLightsBufferInfo, nullptr),
            vk::WriteDescriptorSet(descriptorSet, 6, 0, 1,
                                   vk::DescriptorType::eStorageBuffer, nullptr,
                                   &outLightSamplersBufferInfo, nullptr),
            vk::WriteDescriptorSet(descriptorSet, 7, 0, textures.size(),
                                   vk::DescriptorType::eCombinedImageSampler, outTexturesInfos.data(),
                                   nullptr, nullptr)
    };
}

nvvkpp::RaytracingBuilderKHR::Blas SceneLoader::modelToBlas(const Model &model) {
    // Setting up the creation info of acceleration structure
    vk::AccelerationStructureCreateGeometryTypeInfoKHR asCreate;
    asCreate.setGeometryType(vk::GeometryTypeKHR::eTriangles);
    asCreate.setIndexType(vk::IndexType::eUint32);
    asCreate.setVertexFormat(vk::Format::eR32G32B32Sfloat);
    asCreate.setMaxPrimitiveCount(model.indices.size() / 3);  // Nb triangles
    asCreate.setMaxVertexCount(model.vertices.size());
    asCreate.setAllowsTransforms(VK_FALSE);  // No adding transformation matrices

    // Building part
    auto infoVB = vk::BufferDeviceAddressInfo(model.vertexBuffer);
    auto infoIB = vk::BufferDeviceAddressInfo(model.indexBuffer);

    vk::DeviceAddress vertexAddress = device.getBufferAddressKHR(&infoVB);
    vk::DeviceAddress indexAddress = device.getBufferAddressKHR(&infoIB);

    vk::AccelerationStructureGeometryTrianglesDataKHR triangles;
    triangles.setVertexFormat(asCreate.vertexFormat);
    triangles.setVertexData(vertexAddress);
    triangles.setVertexStride(sizeof(Vertex));
    triangles.setIndexType(asCreate.indexType);
    triangles.setIndexData(indexAddress);
    triangles.setTransformData({});

    // Setting up the build info of the acceleration
    vk::AccelerationStructureGeometryKHR asGeom;
    asGeom.setGeometryType(asCreate.geometryType);
    asGeom.setFlags(vk::GeometryFlagBitsKHR::eNoDuplicateAnyHitInvocation);
    asGeom.geometry.setTriangles(triangles);

    // The primitive itself
    vk::AccelerationStructureBuildOffsetInfoKHR offset;
    offset.setFirstVertex(0);
    offset.setPrimitiveCount(asCreate.maxPrimitiveCount);
    offset.setPrimitiveOffset(0);
    offset.setTransformOffset(0);

    // Our blas is only one geometry, but could be made of many geometries
    nvvkpp::RaytracingBuilderKHR::Blas blas{};
    blas.asGeometry.emplace_back(asGeom);
    blas.asCreateGeometryInfo.emplace_back(asCreate);
    blas.asBuildOffsetInfo.emplace_back(offset);

    return blas;
}

void SceneLoader::createBottomLevelAS() {
    std::vector<nvvkpp::RaytracingBuilderKHR::Blas> allBlas;
    allBlas.reserve(models.size());

    for (const auto &model : models) {
        allBlas.push_back(modelToBlas(model));
    }

    rtBuilder.buildBlas(allBlas, vk::BuildAccelerationStructureFlagBitsKHR::ePreferFastTrace);
}

void SceneLoader::createTopLevelAS() {
    std::vector<nvvkpp::RaytracingBuilderKHR::Instance> tlas;
    tlas.reserve(models.size());

    for (int i = 0; i < static_cast<int>(instances.size()); ++i) {
        Instance instance = instances[i];

        nvvkpp::RaytracingBuilderKHR::Instance rayInst;
        rayInst.transform = glm::value_ptr(instance.transform);
        rayInst.instanceId = i;
        rayInst.blasId = instance.iModel;
        rayInst.hitGroupId = 0; // Same hit group for all
        rayInst.flags = vk::GeometryInstanceFlagBitsKHR::eTriangleFacingCullDisable;
        rayInst.mask = 0xFF;
        tlas.emplace_back(rayInst);
    }

    rtBuilder.buildTlas(tlas, vk::BuildAccelerationStructureFlagBitsKHR::ePreferFastTrace);
}

const vk::AccelerationStructureKHR &SceneLoader::getAccelerationStructure() {
    return rtBuilder.getAccelerationStructure();
}

void SceneLoader::cleanup() {

    for (auto &texture: textures) {
        texture.cleanup(device);
    }

    device.free(materialBufferMemory);
    device.destroy(materialBuffer);
    device.free(instanceInfoBufferMemory);
    device.destroy(instanceInfoBuffer);
    device.free(lightsBufferMemory);
    device.destroy(lightsBuffer);
    device.free(lightsSamplerBufferMemory);
    device.destroy(lightsSamplersBuffer);

    for (auto &model: models) {
        model.cleanup();
    }

    rtBuilder.destroy();
}

size_t SceneLoader::getModelCount() {
    return models.size();
}
