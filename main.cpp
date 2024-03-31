#include <iostream>
#include <SDL.h>
#include <SDL_image.h>
#include <SDL_mixer.h>
#include <cmath>
#include <vector>
#include <unordered_map>
#include <memory>

class TextureLoader {
public:
    explicit TextureLoader(SDL_Renderer *renderer) : mRenderer(renderer) {
    }

    TextureLoader(const TextureLoader &other) = delete;

    SDL_Texture *load(const std::string &name) {
        if (mCache.count(name) > 0)
            return mCache[name];

        SDL_LogMessage(SDL_LOG_CATEGORY_APPLICATION, SDL_LOG_PRIORITY_INFO, "Loading texture %s", name.c_str());
        SDL_Texture *texture = IMG_LoadTexture(mRenderer, name.c_str());
        mCache[name] = texture;
        return texture;
    }

private:
    SDL_Renderer *mRenderer;
    std::unordered_map<std::string, SDL_Texture *> mCache;
};

class Vec2 {
public:
    double x;
    double y;

    Vec2(double x, double y) : x(x), y(y) {

    }

    void rotate(double radian) {
        double cos = SDL_cos(radian);
        double sin = SDL_sin(radian);
        double nextX = x * cos - y * sin;
        double nextY = x * sin + y * cos;

        x = nextX;
        y = nextY;
    }

    void add(const Vec2 &other, double scale) {
        x += scale * other.x;
        y += scale * other.y;
    }

    void substract(const Vec2 &other) {
        x -= other.x;
        y -= other.y;
    }

    void scale(double factor) {
        x *= factor;
        y *= factor;
    }

    double length() const {
        return SDL_sqrt(x * x + y * y);
    }

    double getRotation() {
        return SDL_atan2(y, x);
    }


    void normalize() {
        double l = length();
        x /= l;
        y /= l;
    }
};

class IGameStage {
public:
    virtual const Vec2 &getPlayerPosition() = 0;

    virtual void addLaser(const Vec2 &position, double angle) = 0;

    virtual const Vec2 &getWorldSize() = 0;
};


class GameEntity {
public:
    SDL_Texture *texture;
    Vec2 position;

    GameEntity(const Vec2 &position) : position(position) {}

    SDL_Rect getRect() {
        SDL_Rect r;
        SDL_QueryTexture(texture, nullptr, nullptr, &r.w, &r.h);
        r.x = int(position.x - r.w / 2);
        r.y = int(position.y - r.h / 2);
        return r;
    }

    virtual void onTick(IGameStage *stage) = 0;
};

class Laser : public GameEntity {
public:
    Vec2 directionVector{0, -1};
    double angle = 0;

    Laser(TextureLoader *textureLoader, const Vec2 &position, double angle) : GameEntity(position), angle{angle} {
        texture = textureLoader->load("/home/levirs565/Unduhan/SpaceShooterRedux/PNG/Lasers/laserBlue01.png");
        directionVector.rotate(angle * M_PI / 180.0);
    }

    bool mustGone = false;

    void onTick(IGameStage *stage) override {
        position.add(directionVector, 7.5);
    }
};

class PlayerShip : public GameEntity {
public:
    enum Direction {
        DIRECTION_UP,
        DIRECTION_DOWN,
        DIRECTION_NONE
    };

    enum Rotation {
        ROTATION_LEFT,
        ROTATION_RIGHT,
        ROTATION_NONE
    };

    Vec2 directionVector{0, 0};
    double angle = 0;
    Uint32 lastFire = 0;

    explicit PlayerShip(TextureLoader *textureLoader, const Vec2 &position) : GameEntity(position) {
        texture = textureLoader->load("/home/levirs565/Unduhan/SpaceShooterRedux/PNG/playerShip3_blue.png");
    }

    void setDirection(Direction direction, Rotation rotation) {
        if (direction == DIRECTION_UP) {
            directionVector.y = -1;
            directionVector.x = 0;
        } else if (direction == DIRECTION_DOWN) {
            directionVector.x = 0;
            directionVector.y = 1;
        } else {
            directionVector.x = 0;
            directionVector.y = 0;
        }

        if (rotation == ROTATION_LEFT)
            angle -= 5;

        if (rotation == ROTATION_RIGHT)
            angle += 5;

        directionVector.rotate(angle * M_PI / 180.0);
    }

    void onTick(IGameStage *stage) override {
        position.add(directionVector, 5);

        const Vec2 &worldSize = stage->getWorldSize();
        position.x = SDL_clamp(position.x, 0, worldSize.x);
        position.y = SDL_clamp(position.y, 0, worldSize.y);
    }

    void doFire(IGameStage *stage) {
        if ((SDL_GetTicks() - lastFire >= 500)) {
            stage->addLaser(position, angle);
            lastFire = SDL_GetTicks();
        }
    }
};

class Enemy : public GameEntity {
public:
    Vec2 velocity{0, 0};
    Uint32 lastFire = 0;
    double angle = 0;

    Enemy(TextureLoader *textureLoader, const Vec2 &position) : GameEntity(position) {
        texture = textureLoader->load("/home/levirs565/Unduhan/SpaceShooterRedux/PNG/Enemies/enemyBlack1.png");
    }

    bool isHit = false;

    void onTick(IGameStage *stage) override {
        Vec2 desiredVelocity = Vec2(stage->getPlayerPosition());
        desiredVelocity.substract(position);

        double distance = desiredVelocity.length();

        desiredVelocity.normalize();
        desiredVelocity.scale(2);

        Vec2 direction(desiredVelocity);

        double slowingDistance = 100;
        double stopRadius = 200;
        bool canAttack = false;

        if (distance < slowingDistance + stopRadius) {
            canAttack = true;
            desiredVelocity.scale((std::max(distance - stopRadius, 0.0)) / slowingDistance);
        }

        Vec2 steering = Vec2(desiredVelocity);

        steering.substract(velocity);
        direction.substract(velocity);

        direction.add(velocity, 1);
        velocity.add(steering, 1);

        position.add(velocity, 1);

        angle = direction.getRotation() * 180.0 / M_PI - 90;

        if (SDL_GetTicks() - lastFire >= 1000 && canAttack) {
            SDL_Rect enemyRect = getRect();
            Vec2 laserPos(0, enemyRect.h);
            laserPos.rotate((angle) * M_PI / 180.0);
            laserPos.add(position, 1);
            stage->addLaser(laserPos, angle - 180);
            lastFire = SDL_GetTicks();
        }
    }
};

class App : public IGameStage {
public:
    App() {
        if (SDL_Init(SDL_INIT_VIDEO) < 0) {
            std::cout << "Initializing SDL failed" << std::endl;
            exit(1);
        }
        IMG_Init(IMG_INIT_JPG | IMG_INIT_PNG);
        if (Mix_OpenAudio(44100, MIX_DEFAULT_FORMAT, 2, 1024) == 1) {
            std::cout << "Initializing mixer failed" << std::endl;
            exit(1);
        }

        mWindow = SDL_CreateWindow(
                "Space",
                SDL_WINDOWPOS_UNDEFINED,
                SDL_WINDOWPOS_UNDEFINED,
                800,
                600,
                0);

        if (!mWindow) {
            std::cout << "Initializing window failed" << std::endl;
            exit(1);
        }

        SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "linear");

        mRenderer = SDL_CreateRenderer(mWindow, -1, SDL_RENDERER_ACCELERATED);

        if (!mRenderer) {
            std::cout << "Initializing renderer failed" << std::endl;
            exit(1);
        }

        mTextureLoader = std::make_unique<TextureLoader>(mRenderer);
        mBackgroundTexture = mTextureLoader->load("/home/levirs565/Unduhan/SpaceShooterRedux/Backgrounds/black.png");

        mLaserSound = Mix_LoadWAV("/home/levirs565/Unduhan/SpaceShooterRedux/Bonus/sfx_laser1.ogg");

        mPlayerShip = std::make_unique<PlayerShip>(mTextureLoader.get(), Vec2(400, 400));
        mEnemyList.push_back(std::move(std::make_unique<Enemy>(mTextureLoader.get(), Vec2(100, 100))));
    }

    void processKeyDown(const SDL_KeyboardEvent &key) {
        if (key.repeat != 0) return;

        if (key.keysym.scancode == SDL_SCANCODE_UP)
            mIsUp = true;

        if (key.keysym.scancode == SDL_SCANCODE_DOWN)
            mIsDown = true;

        if (key.keysym.scancode == SDL_SCANCODE_LEFT)
            mIsLeft = true;

        if (key.keysym.scancode == SDL_SCANCODE_RIGHT)
            mIsRight = true;

        if (key.keysym.scancode == SDL_SCANCODE_LCTRL)
            mIsFire = true;
    }

    void processKeyUp(const SDL_KeyboardEvent &key) {
        if (key.repeat != 0) return;

        if (key.keysym.scancode == SDL_SCANCODE_UP)
            mIsUp = false;

        if (key.keysym.scancode == SDL_SCANCODE_DOWN)
            mIsDown = false;

        if (key.keysym.scancode == SDL_SCANCODE_LEFT)
            mIsLeft = false;

        if (key.keysym.scancode == SDL_SCANCODE_RIGHT)
            mIsRight = false;

        if (key.keysym.scancode == SDL_SCANCODE_LCTRL)
            mIsFire = false;
    }

    void processInput() {
        SDL_Event event;

        while (SDL_PollEvent(&event)) {
            switch (event.type) {
                case SDL_QUIT:
                    exit(0);
                    break;
                case SDL_KEYDOWN:
                    processKeyDown(event.key);
                    break;
                case SDL_KEYUP:
                    processKeyUp(event.key);
                    break;
                default:
                    break;
            }
        }
    }

    void prepareScene() {
        SDL_SetRenderDrawColor(mRenderer, 96, 128, 255, 255);
        SDL_RenderClear(mRenderer);
    }

    void presentScene() {
        SDL_RenderPresent(mRenderer);
    }

    void run() {
        while (true) {
            prepareScene();
            processInput();

            mPlayerShip->setDirection(
                    mIsUp ? PlayerShip::DIRECTION_UP
                          : mIsDown ? PlayerShip::DIRECTION_DOWN
                                    : PlayerShip::DIRECTION_NONE,
                    mIsLeft ? PlayerShip::ROTATION_LEFT
                            : mIsRight ? PlayerShip::ROTATION_RIGHT
                                       : PlayerShip::ROTATION_NONE);


            mPlayerShip->onTick(this);

            if (mIsFire)
                mPlayerShip->doFire(this);

            mCameraPosition.x = SDL_clamp(mPlayerShip->position.x - mCameraSize.x / 2, 0, mWordSize.x - mCameraSize.x);
            mCameraPosition.y = SDL_clamp(mPlayerShip->position.y - mCameraSize.y / 2, 0, mWordSize.y - mCameraSize.y);

            int backgroundWidth, backgroundHeight;
            SDL_QueryTexture(mBackgroundTexture, nullptr, nullptr, &backgroundWidth, &backgroundHeight);

            double backgroundStartY = mCameraPosition.y - fmod(mCameraPosition.y, double(backgroundHeight));
            double backgroundStartX = mCameraPosition.x - fmod(mCameraPosition.x, double(backgroundWidth));
            int backgroundCountY = int(
                    ceil((mCameraPosition.y + mCameraSize.y - backgroundStartY) / double(backgroundHeight)));
            int backgroundCountX = int(
                    ceil((mCameraPosition.x + mCameraSize.x - backgroundStartX) / double(backgroundWidth)));

            for (int backgroundRow = 0; backgroundRow <= backgroundCountY; backgroundRow++) {
                for (int backgroundColumn = 0; backgroundColumn <= backgroundCountX; backgroundColumn++) {
                    blit(mBackgroundTexture, backgroundStartX + backgroundColumn * backgroundWidth,
                         backgroundStartY + backgroundRow * backgroundHeight, 0.0);
                }
            }

            blit(mPlayerShip->texture, mPlayerShip->position.x, mPlayerShip->position.y, mPlayerShip->angle);


            for (const std::unique_ptr<Laser> &laser: mLaserList) {
                if (laser->mustGone) continue;

                laser->onTick(this);
            }

            for (const std::unique_ptr<Enemy> &enemy: mEnemyList) {
                if (enemy->isHit) continue;

                enemy->onTick(this);
            }

            for (const std::unique_ptr<Laser> &laser: mLaserList) {
                if (laser->mustGone) continue;

                for (const std::unique_ptr<Enemy> &enemy: mEnemyList) {
                    if (enemy->isHit) continue;

                    SDL_Rect enemyRect = enemy->getRect();
                    SDL_Rect laserRect = laser->getRect();
                    if (SDL_HasIntersection(&enemyRect, &laserRect)) {
                        laser->mustGone = true;
                        enemy->isHit = true;
                    }
                }
                if (laser->mustGone) continue;

                blit(laser->texture, laser->position.x, laser->position.y, laser->angle);
            }

            for (const std::unique_ptr<Enemy> &enemy: mEnemyList) {
                if (enemy->isHit) continue;

                blit(enemy->texture, enemy->position.x, enemy->position.y, enemy->angle);
            }

            presentScene();
            SDL_Delay(16);
        }
    }


    void blit(SDL_Texture *texture, double x, double y, double angle) {
        SDL_Rect rect;
        rect.x = int(x - mCameraPosition.x);
        rect.y = int(y - mCameraPosition.y);
        SDL_QueryTexture(texture, NULL, NULL, &rect.w, &rect.h);
        rect.x -= rect.w / 2;
        rect.y -= rect.h / 2;
        SDL_RenderCopyEx(mRenderer, texture, NULL, &rect, angle, NULL, SDL_FLIP_NONE);
    }

    const Vec2 &getPlayerPosition() override {
        return mPlayerShip->position;
    }

    void addLaser(const Vec2 &position, double angle) override {
        std::unique_ptr<Laser> laser = std::make_unique<Laser>(mTextureLoader.get(), position,
                                                               angle);
        mLaserList.push_back(std::move(laser));
        Mix_PlayChannel(1, mLaserSound, 0);
    }

    const Vec2 &getWorldSize() override {
        return mWordSize;
    }

private:
    SDL_Renderer *mRenderer;
    SDL_Window *mWindow;
    SDL_Texture *mBackgroundTexture;
    std::unique_ptr<PlayerShip> mPlayerShip;
    std::unique_ptr<TextureLoader> mTextureLoader;
    std::vector<std::unique_ptr<Laser>> mLaserList;
    std::vector<std::unique_ptr<Enemy>> mEnemyList;
    Vec2 mCameraSize{800, 600};
    Vec2 mCameraPosition{0, 0};
    Vec2 mWordSize{5000, 5000};
    Mix_Chunk *mLaserSound;
    bool mIsUp = false;
    bool mIsLeft = false;
    bool mIsDown = false;
    bool mIsRight = false;
    bool mIsFire = false;
};

int main() {
    App app;
    app.run();
    return 0;
}