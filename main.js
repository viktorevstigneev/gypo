// setTimeout(() => {
//   document.querySelector(".loader").style.display = "none";
// }, 3000);

// Graphics variables
let container;
let camera, controls, scene, renderer;
let textureLoader;
const clock = new THREE.Clock();

const gvSetting = document.querySelector(".ss_gravity").dataset.setting;

// Physics variables
const gravityConstant = -parseFloat(gvSetting);

let collisionConfiguration;
let dispatcher;
let broadphase;
let solver;
let softBodySolver;
let physicsWorld;
const rigidBodies = [];
const margin = 0.05;
let hinge;
let rope;
let transformAux1;
let medal;
let medalBody;
let house;

let m1Body;
let m2Body;

Ammo().then(function (AmmoLib) {
  Ammo = AmmoLib;

  init();
  animate();
});

function init() {
  initGraphics();

  initPhysics();

  createObjects();

  initInput();
}

// ---------------------------- INIT GRAPHICS -----------------------------

function initGraphics() {
  container = document.getElementById("container");

  camera = new THREE.PerspectiveCamera(
    75,
    window.innerWidth / window.innerHeight,
    0.9,
    100
  );

  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x404040);

  camera.position.set(0, 0, 8);

  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.shadowMap.enabled = true;
  // renderer.physicallyCorrectLights = true;
  container.appendChild(renderer.domElement);
  //OrbitControls
  controls = new THREE.OrbitControls(camera, renderer.domElement);

  textureLoader = new THREE.TextureLoader();

  const light = new THREE.PointLight(0xffffff, 3);
  light.position.set(0, 4, 5);

  scene.add(light);
}

// ----------------------------------------INIT PHISICS--------------------------------------------
function initPhysics() {
  // Physics configuration

  collisionConfiguration = new Ammo.btSoftBodyRigidBodyCollisionConfiguration();
  dispatcher = new Ammo.btCollisionDispatcher(collisionConfiguration);
  broadphase = new Ammo.btDbvtBroadphase();
  solver = new Ammo.btSequentialImpulseConstraintSolver();
  softBodySolver = new Ammo.btDefaultSoftBodySolver();
  physicsWorld = new Ammo.btSoftRigidDynamicsWorld(
    dispatcher,
    broadphase,
    solver,
    collisionConfiguration,
    softBodySolver
  );
  physicsWorld.setGravity(new Ammo.btVector3(0, gravityConstant, 0));
  physicsWorld
    .getWorldInfo()
    .set_m_gravity(new Ammo.btVector3(0, gravityConstant, 0));

  transformAux1 = new Ammo.btTransform();
}

// ------------------------------------------------CREATE OBJECTS -----------------------------
async function createObjects() {
  function createConvexShape(convexGeometry) {
    const points = [];

    for (
      let i = 0;
      i < convexGeometry.attributes.position.array.length;
      i += 3
    ) {
      points.push(
        new Ammo.btVector3(
          convexGeometry.attributes.position.array[i],
          convexGeometry.attributes.position.array[i + 1],
          convexGeometry.attributes.position.array[i + 2]
        )
      );
    }

    const shape = new Ammo.btConvexHullShape();
    points.forEach((point) => {
      shape.addPoint(point);
    });

    return shape;
  }

  function loadModel(url) {
    return new Promise((resolve, reject) => {
      gltfLoader.load(url, (loadedModel) => {
        resolve(loadedModel);
      });
    });
  }

  const pos = new THREE.Vector3();
  const quat = new THREE.Quaternion();

  const gltfLoader = new THREE.GLTFLoader();
  // ------------------------- BORDERS --------------------------------------
  pos.set(0, -5.5, 0);
  quat.set(0, 0, 0, 1);
  const ground = createParalellepiped(
    40,
    0.1,
    40,
    0,
    pos,
    quat,
    new THREE.MeshPhongMaterial({ color: 0xffffff })
  );

  pos.set(0, 7, 0);
  quat.set(0, 0, 0, 1);
  const ground1 = createParalellepiped(
    20,
    0.1,
    20,
    0,
    pos,
    quat,
    new THREE.MeshPhongMaterial({ color: 0xffffff })
  );

  pos.set(0, 0, -7);
  quat.set(0.7, 0, 0, 1);
  const ground2 = createParalellepiped(
    20,
    0.1,
    20,
    0,
    pos,
    quat,
    new THREE.MeshPhongMaterial({ color: 0xffffff })
  );

  pos.set(-window.innerWidth / 7.12 / 11, 0, 0);
  quat.set(0, 0, -0.7, 1);
  const ground3 = createParalellepiped(
    20,
    0.1,
    20,
    0,
    pos,
    quat,
    new THREE.MeshPhongMaterial({ color: 0xffffff })
  );

  pos.set(window.innerWidth / 7.12 / 11, 0, 0);
  quat.set(0, 0, 0.7, 1);
  const ground4 = createParalellepiped(
    20,
    0.1,
    20,
    0,
    pos,
    quat,
    new THREE.MeshPhongMaterial({ color: 0xffffff })
  );

  pos.set(0, -5, 4);
  quat.set(0.7, 0, 0, 1);
  const ground5 = createParalellepiped(
    20,
    0.1,
    1,
    0,
    pos,
    quat,
    new THREE.MeshPhongMaterial({ color: 0xffffff })
  );

  // ground.visible = false;
  // ground1.visible = false;
  // ground2.visible = false;
  // ground3.visible = false;
  // ground4.visible = false;
  // ground5.visible = false;

  // ------------------------------------------------Model 2--------------------------
  let m2 = await loadModel(document.querySelector(".m2_mesh").dataset.setting);

  m2.scene.scale.x = 0.5;
  m2.scene.scale.y = 0.5;
  m2.scene.scale.z = 0.5;

  m2.scene.position.set(window.innerWidth / 200, 1, -1);
  scene.add(m2.scene);

  let verticesM2 = m2.scene.children[0].geometry;

  const m2Shape = createConvexShape(verticesM2);

  pos.set(window.innerWidth / 200, 0, -2);
  quat.set(0, 0, 0, 1);

  createRigidBody(m2.scene, m2Shape, 1, pos, quat, false, false, true);


  // // ------------------------------------------------Model 1--------------------------
  let m1 = await loadModel(document.querySelector(".m1_mesh").dataset.setting);

  m1.scene.scale.x = 0.5;
  m1.scene.scale.y = 0.5;
  m1.scene.scale.z = 0.5;

  // ---------------------------------------------------- VARIANT 1 ---------------------------------------------

  m1.scene.position.set(-window.innerWidth / 200, -2, -0.5);
  scene.add(m1.scene);

  let vertices = m1.scene.children[0].geometry;

  const m1Shape = createConvexShape(vertices);

  pos.set(-window.innerWidth / 200, -2, -0.5);
  quat.set(0, 0, 0, 1);

  createRigidBody(m1.scene, m1Shape, 1, pos, quat, false, true);

  // ---------------------------------------------------- VARIANT 2 ---------------------------------------------
  // let vertices = [];
  // m1.scene.traverse(function (child) {
  //   if (child.isMesh) {
  //     let geometry = child.geometry;
  //     geometry.computeBoundingBox(); // Обновляем ограничивающий параллелепипед для получения вершин модели
  //     let positionAttribute = geometry.attributes.position;
  //     for (let i = 0; i < positionAttribute.count; i++) {
  //       let point = new Ammo.btVector3(
  //         positionAttribute.getX(i),
  //         positionAttribute.getY(i),
  //         positionAttribute.getZ(i)
  //       );
  //       vertices.push(point);
  //     }
  //   }
  // });

  // let m1Shape = new Ammo.btConvexHullShape();
  // for (let i = 0; i < vertices.length; i++) {
  //   m1Shape.addPoint(vertices[i]);
  // }

  // pos.set(-window.innerWidth / 200, -2, -0.5);
  // quat.set(0, 0, 0, 1);

  // createRigidBody(m1.scene, m1Shape, 1, pos, quat, false, true);
  // ------------------------------------------MEDAL -------------------
  const medalMass = 25.2;
  const medalRadius = 3.3;

  medal = await loadModel(
    document.querySelector(".ss_medal_mesh").dataset.setting
  );

  medal.castShadow = true;
  medal.receiveShadow = true;
  const medalShape = new Ammo.btSphereShape(medalRadius);
  medalShape.setMargin(margin);
  pos.set(0, 0, 0);
  quat.set(0, 0, 0, 1);

  createRigidBody(medal.scene, medalShape, medalMass, pos, quat, true);

  medal.scene.scale.x = 0.635;
  medal.scene.scale.y = 0.635;
  medal.scene.scale.z = 0.635;
  medal.scene.userData.physicsBody.setFriction(0.5);

  // -------------------------------- HOUSE -----------------------------------------------

  const textureLoader = new THREE.TextureLoader();
  const texture = textureLoader.load(
    document.querySelector(".ss_box_albedo").dataset.setting
  );

  house = await loadModel(
    document.querySelector(".ss_box_mesh").dataset.setting
  );

  house.scene.traverse((child) => {
    if (child.isMesh) {
      child.material.map = texture;
    }
  });

  const boxWidth = 7.12;
  const diff = window.innerWidth / boxWidth;
  house.scene.scale.x = diff / 50;
  house.scene.scale.z = diff / 40;

  scene.add(house.scene);

  // ------------------------------------------------ROPE----------------------------------------
  const ropeNumSegments = 10;
  const ropeLength = 5;
  const ropeMass = 3;
  const ropePos = medal.scene.position.clone();
  console.log("ropePos: ", ropePos);
  ropePos.y += medalRadius - 2;

  const segmentLength = ropeLength / ropeNumSegments;
  const ropeGeometry = new THREE.BufferGeometry();
  const ropeMaterial = new THREE.LineBasicMaterial({ color: 0xd8c9b8 });
  const ropePositions = [];
  const ropeIndices = [];

  for (let i = 0; i < ropeNumSegments + 1; i++) {
    ropePositions.push(ropePos.x, ropePos.y + i * segmentLength, ropePos.z);
  }

  for (let i = 0; i < ropeNumSegments; i++) {
    ropeIndices.push(i, i + 1);
  }

  ropeGeometry.setIndex(
    new THREE.BufferAttribute(new Uint16Array(ropeIndices), 1)
  );
  ropeGeometry.setAttribute(
    "position",
    new THREE.BufferAttribute(new Float32Array(ropePositions), 3)
  );
  ropeGeometry.computeBoundingSphere();
  rope = new THREE.LineSegments(ropeGeometry, ropeMaterial);

  rope.castShadow = true;
  rope.receiveShadow = true;
  scene.add(rope);

  // -----------------------------------------ROPE PHISICS ------------------
  const softBodyHelpers = new Ammo.btSoftBodyHelpers();
  const ropeStart = new Ammo.btVector3(ropePos.x, ropePos.y, ropePos.z);
  const ropeEnd = new Ammo.btVector3(
    ropePos.x,
    ropePos.y + ropeLength,
    ropePos.z
  );
  const ropeSoftBody = softBodyHelpers.CreateRope(
    physicsWorld.getWorldInfo(),
    ropeStart,
    ropeEnd,
    ropeNumSegments - 1,
    0
  );
  const sbConfig = ropeSoftBody.get_m_cfg();
  sbConfig.set_viterations(10);
  sbConfig.set_piterations(10);
  ropeSoftBody.setTotalMass(ropeMass, false);
  Ammo.castObject(ropeSoftBody, Ammo.btCollisionObject)
    .getCollisionShape()
    .setMargin(margin * 3);
  physicsWorld.addSoftBody(ropeSoftBody, 1, -1);
  rope.userData.physicsBody = ropeSoftBody;
  // Disable deactivation
  ropeSoftBody.setActivationState(4);

  // // ------------------------- BORDERS --------------------------------------
  // pos.set(0, -5.5, 0);
  // quat.set(0, 0, 0, 1);
  // const ground = createParalellepiped(
  //   40,
  //   0.1,
  //   40,
  //   0,
  //   pos,
  //   quat,
  //   new THREE.MeshPhongMaterial({ color: 0xffffff })
  // );

  // pos.set(0, 7, 0);
  // quat.set(0, 0, 0, 1);
  // const ground1 = createParalellepiped(
  //   20,
  //   0.1,
  //   20,
  //   0,
  //   pos,
  //   quat,
  //   new THREE.MeshPhongMaterial({ color: 0xffffff })
  // );

  // pos.set(0, 0, -7);
  // quat.set(0.7, 0, 0, 1);
  // const ground2 = createParalellepiped(
  //   20,
  //   0.1,
  //   20,
  //   0,
  //   pos,
  //   quat,
  //   new THREE.MeshPhongMaterial({ color: 0xffffff })
  // );

  // pos.set(-window.innerWidth / 7.12 / 11, 0, 0);
  // quat.set(0, 0, -0.7, 1);
  // const ground3 = createParalellepiped(
  //   20,
  //   0.1,
  //   20,
  //   0,
  //   pos,
  //   quat,
  //   new THREE.MeshPhongMaterial({ color: 0xffffff })
  // );

  // pos.set(window.innerWidth / 7.12 / 11, 0, 0);
  // quat.set(0, 0, 0.7, 1);
  // const ground4 = createParalellepiped(
  //   20,
  //   0.1,
  //   20,
  //   0,
  //   pos,
  //   quat,
  //   new THREE.MeshPhongMaterial({ color: 0xffffff })
  // );

  // pos.set(0, -5, 0);
  // quat.set(0.7, 0, 0, 1);
  // const ground5 = createParalellepiped(
  //   20,
  //   0.1,
  //   1,
  //   0,
  //   pos,
  //   quat,
  //   new THREE.MeshPhongMaterial({ color: 0xffffff })
  // );

  // ground.visible = false;
  // ground1.visible = false;
  // ground2.visible = false;
  // ground3.visible = false;
  // ground4.visible = false;
  // ground5.visible = false;

  // ----------------------------------UP POINT -----------------------------------

  function createParalellepiped(sx, sy, sz, mass, pos, quat, material) {
    const threeObject = new THREE.Mesh(
      new THREE.BoxGeometry(sx, sy, sz, 1, 1, 1),
      material
    );
    const shape = new Ammo.btBoxShape(
      new Ammo.btVector3(sx * 0.5, sy * 0.5, sz * 0.5)
    );
    shape.setMargin(margin);

    createRigidBody(threeObject, shape, mass, pos, quat);

    return threeObject;
  }

  const baseMaterial = new THREE.MeshPhongMaterial({ color: 0x606060 });

  const arm = createParalellepiped(0, 0, 0, 0, pos, quat, baseMaterial);

  // arm.castShadow = true;
  // arm.receiveShadow = true;
  // arm.visible = false;

  //---------------------------------------CONNECTING OBJ-----------------------------------------
  const influence = 1;

  ropeSoftBody.appendAnchor(
    0,
    medal.scene.userData.physicsBody,
    true,
    influence
  );
  ropeSoftBody.appendAnchor(
    ropeNumSegments,
    arm.userData.physicsBody,
    true,
    influence
  );

  // --------------------------------------------HOUSE AND ARM ------------------------------------------
  // const pivotA = new Ammo.btVector3(0,  0.5, 0);
  // const pivotB = new Ammo.btVector3(0, -0.2, - 0.5);
  // const axis = new Ammo.btVector3(0, 1, 0);
  // hinge = new Ammo.btHingeConstraint(
  //   pylon.userData.physicsBody,
  //   arm.userData.physicsBody,
  //   pivotA,
  //   pivotB,
  //   axis,
  //   axis,
  //   true
  // );
  // physicsWorld.addConstraint(hinge, true);
}

function createRigidBody(
  threeObject,
  physicsShape,
  mass,
  pos,
  quat,
  isModel,
  isM1,
  isM2
) {
  threeObject.position.copy(pos);
  threeObject.quaternion.copy(quat);

  const transform = new Ammo.btTransform();
  transform.setIdentity();
  transform.setOrigin(new Ammo.btVector3(pos.x, pos.y, pos.z));
  transform.setRotation(new Ammo.btQuaternion(quat.x, quat.y, quat.z, quat.w));
  const motionState = new Ammo.btDefaultMotionState(transform);

  const localInertia = new Ammo.btVector3(0, 0, 0);
  physicsShape.calculateLocalInertia(mass, localInertia);

  const rbInfo = new Ammo.btRigidBodyConstructionInfo(
    mass,
    motionState,
    physicsShape,
    localInertia
  );
  const body = new Ammo.btRigidBody(rbInfo);

  threeObject.userData.physicsBody = body;

  scene.add(threeObject);

  if (mass > 0) {
    rigidBodies.push(threeObject);

    // Disable deactivation
    body.setActivationState(4);
  }

  physicsWorld.addRigidBody(body);

  if (isModel) {
    medalBody = body;
  }

  if (isM1) {
    m1Body = body;
  }
  if (isM2) {
    m2Body = body;
  }
}

function initInput() {
  // Инициализация переменных для скользящего среднего
  let newAcceleration;
  let averageAcceleration = new Ammo.btVector3(0, 0, 0);
  let maxAcceleration = +document.querySelector(".scene__maxAcceleration")
    .dataset.setting; // Максимальное ускорение, можно настроить под свои потребности

  // document.querySelector('.scene_averageAcceleration').dataset.setting
  let dampingFactor = +document.querySelector(".scene__dampingFactor").dataset
    .setting; // Фактор затухания, можно настроить под свои потребности

  // Инициализация переменных
  let initialPosition = new Ammo.btVector3(0, 0, 0); // Предположим, что изначальное положение медали находится в центре (0, 0, 0)
  let idleThreshold = +document.querySelector(".scene__idleThreshold").dataset
    .setting; // Порог бездействия, например, 3 секунды

  let lastActivityTime = Date.now(); // Время последнего действия

  let initialBeta = null;

  window.addEventListener("deviceorientation", (event) => {
    if (initialBeta === null) {
      initialBeta = event.beta;
    }

    let gyroscopeData = {
      alpha: event.alpha,
      beta: event.beta - initialBeta,
      gamma: event.gamma,
    };

    newAcceleration = new Ammo.btVector3(
      gyroscopeData.gamma,
      gyroscopeData.alpha,
      gyroscopeData.beta
    );

    averageAcceleration.op_mul(dampingFactor);
    averageAcceleration.op_add(newAcceleration);

    // Ограничение максимального ускорения
    if (averageAcceleration.length() > maxAcceleration) {
      averageAcceleration.op_mul(
        maxAcceleration / averageAcceleration.length()
      );
    }

    // ------------------------------------------------Применение ускорения к медали----------------------------
    var currentVelocity = medalBody?.getLinearVelocity();

    var newVelocity = new Ammo.btVector3(
      currentVelocity?.x() + averageAcceleration.x(),
      currentVelocity?.y(),
      currentVelocity?.z() + averageAcceleration.z()
    );
    medalBody.setLinearVelocity(newVelocity);

    // ----------------------------------------------Применение ускорения к м1----------------------------------
    var currentVelocity1 = m1Body?.getLinearVelocity();
    var newVelocity1 = new Ammo.btVector3(
      currentVelocity1?.x() + averageAcceleration.x(),
      currentVelocity1?.y(),
      currentVelocity1?.z() + averageAcceleration.z()
    );
    m1Body.setLinearVelocity(newVelocity1);

    // -----------------------------------------------Применение ускорения к м2---------------------------------
    var currentVelocity2 = m2Body?.getLinearVelocity();
    var newVelocity2 = new Ammo.btVector3(
      currentVelocity2?.x() + averageAcceleration.x(),
      currentVelocity2?.y(),
      currentVelocity2?.z() + averageAcceleration.z()
    );
    m2Body.setLinearVelocity(newVelocity2);

    lastActivityTime = Date.now();
  });

  // ---------------------------------------MEDAL RETURN BACK-----------------------------
  // Функция для проверки бездействия и возвращения медали в исходное положение
  function checkIdleAndReturnToInitial() {
    if (Date.now() - lastActivityTime > idleThreshold) {
      // Медаль не трогали в течение достаточного времени, возвращаем её в исходное положение

      var currentPosition = medalBody?.getWorldTransform().getOrigin();

      // Вычисляем ускорение, направленное к исходной позиции
      var accelerationToInitial = new Ammo.btVector3(
        (initialPosition.x() - currentPosition?.x()) * 0.1, // Направляем ускорение к исходной позиции с учетом плавного изменения
        (initialPosition.y() - currentPosition?.y()) * 0.1,
        (initialPosition.z() - currentPosition?.z()) * 0.1
      );

      // Применяем ускорение к медали
      var currentVelocity = medalBody?.getLinearVelocity();
      var newVelocity = new Ammo.btVector3(
        accelerationToInitial.x(),
        accelerationToInitial.y(),
        accelerationToInitial.z()
      );
      medalBody?.setLinearVelocity(newVelocity);
    }

    // Повторяем проверку через короткий интервал времени
    setTimeout(checkIdleAndReturnToInitial, 3000);
  }

  checkIdleAndReturnToInitial();
}

function animate() {
  requestAnimationFrame(animate);
  // controls.update();
  render();
}

function render() {
  const deltaTime = clock.getDelta();

  updatePhysics(deltaTime);

  renderer.render(scene, camera);
}

function updatePhysics(deltaTime) {
  // Hinge control
  // Step world
  physicsWorld.stepSimulation(deltaTime, 60);

  // Update rope
  const softBody = rope?.userData.physicsBody;

  const ropePositions = rope?.geometry.attributes.position.array;
  const numVerts = ropePositions?.length / 3;

  const nodes = softBody?.get_m_nodes();
  let indexFloat = 0;

  for (let i = 0; i < numVerts; i++) {
    const node = nodes.at(i);
    const nodePos = node.get_m_x();
    ropePositions[indexFloat++] = nodePos.x();
    ropePositions[indexFloat++] = nodePos.y();
    ropePositions[indexFloat++] = nodePos.z();
  }

  if (rope) {
    rope.geometry.attributes.position.needsUpdate = true;
  }

  // if (medal) {
  //   const nodes = softBody?.get_m_nodes();
  //   let indexFloat = 0;
  //   const node = nodes.at(numVerts - 1);
  //   const nodePos = node.get_m_x();
  //   medal.scene.position.set(nodePos.x(), nodePos.y(), nodePos.z());
  // }

  // Update rigid bodies
  for (let i = 0, il = rigidBodies.length; i < il; i++) {
    const objThree = rigidBodies[i];
    const objPhys = objThree.userData.physicsBody;
    const ms = objPhys.getMotionState();
    if (ms) {
      ms.getWorldTransform(transformAux1);
      const p = transformAux1.getOrigin();
      const q = transformAux1.getRotation();
      objThree.position.set(p.x(), p.y(), p.z());
      objThree.quaternion.set(q.x(), q.y(), q.z(), q.w());
    }
  }
}
