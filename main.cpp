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
};

class Laser {
public:
    SDL_Texture *texture;
    Vec2 position{0, 0};
    Vec2 directionVector{0, -1};
    double angle = 0;

    Laser(TextureLoader *textureLoader, const Vec2 &position, double angle) : angle{angle}, position{position} {
        texture = textureLoader->load("/home/levirs565/Unduhan/SpaceShooterRedux/PNG/Lasers/laserBlue01.png");
        directionVector.rotate(angle * M_PI / 180.0);
    }
};

class App {
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
        mSpaceShipTexture = mTextureLoader->load("/home/levirs565/Unduhan/SpaceShooterRedux/PNG/playerShip3_blue.png");

        mLaserSound = Mix_LoadWAV("/home/levirs565/Unduhan/SpaceShooterRedux/Bonus/sfx_laser1.ogg");
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

        if (key.keysym.scancode == SDL_SCANCODE_SPACE)
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

        if (key.keysym.scancode == SDL_SCANCODE_SPACE)
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

            if (mIsUp) {
                mDirectionVector.y = -1;
                mDirectionVector.x = 0;
            } else if (mIsDown) {
                mDirectionVector.x = 0;
                mDirectionVector.y = 1;
            } else {
                mDirectionVector.x = 0;
                mDirectionVector.y = 0;
            }

            if (mIsLeft)
                mRotation -= 5;

            if (mIsRight)
                mRotation += 5;

            mDirectionVector.rotate(mRotation * M_PI / 180.0);

            mPlayerPosition.add(mDirectionVector, 5);

            blit(mSpaceShipTexture, int(mPlayerPosition.x), int(mPlayerPosition.y), mRotation);

            if (mIsFire && (SDL_GetTicks() - mLastFire >= 150)) {
                std::unique_ptr<Laser> laser = std::make_unique<Laser>(mTextureLoader.get(), mPlayerPosition,
                                                                       mRotation);
                mLaserList.push_back(std::move(laser));
                mLastFire = SDL_GetTicks();
                Mix_PlayChannel(1, mLaserSound, 0);
            }

            for (const std::unique_ptr<Laser> &laser: mLaserList) {
                laser->position.add(laser->directionVector, 15);
                blit(laser->texture, int(laser->position.x), int(laser->position.y), laser->angle);
            }

            presentScene();
            SDL_Delay(16);
        }
    }


    void blit(SDL_Texture *texture, int x, int y, double angle) {
        SDL_Rect rect;
        rect.x = x;
        rect.y = y;
        SDL_QueryTexture(texture, NULL, NULL, &rect.w, &rect.h);
        rect.x -= rect.w / 2;
        rect.y -= rect.h / 2;
        SDL_RenderCopyEx(mRenderer, texture, NULL, &rect, angle, NULL, SDL_FLIP_NONE);
    }

private:
    SDL_Renderer *mRenderer;
    SDL_Window *mWindow;
    SDL_Texture *mSpaceShipTexture;
    std::unique_ptr<TextureLoader> mTextureLoader;
    std::vector<std::unique_ptr<Laser>> mLaserList;
    Vec2 mPlayerPosition{100, 100};
    Vec2 mDirectionVector{0, 0};
    Mix_Chunk *mLaserSound;
    Uint32 mLastFire = 0;
    double mRotation = 0;
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