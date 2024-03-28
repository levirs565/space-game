#include <iostream>
#include <SDL.h>
#include <SDL_image.h>
#include <cmath>

class Vector2 {
public:
    double x;
    double y;

    Vector2(double x, double y) : x(x), y(y) {

    }

    void rotate(double radian) {
        double cos = SDL_cos(radian);
        double sin = SDL_sin(radian);
        double nextX = x * cos - y * sin;
        double nextY = x * sin + y * cos;

        x = nextX;
        y = nextY;
    }

    void add(const Vector2& other, double scale) {
        x += scale * other.x;
        y += scale * other.y;
    }
};

class Laser {
public:
    SDL_Texture* texture = nullptr;
    Vector2 position{0, 0};
    Vector2 directionVector{0, 0};
    double angle = 0;

    Laser() {}
};

class App {
public:
    App() {
        if (SDL_Init(SDL_INIT_VIDEO) < 0) {
            std::cout << "Initializing SDL failed" << std::endl;
            exit(1);
        }
        IMG_Init(IMG_INIT_JPG | IMG_INIT_PNG);

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

        mRenderer = SDL_CreateRenderer(mWindow, -1,  SDL_RENDERER_ACCELERATED);

        if (!mRenderer) {
            std::cout << "Initializing renderer failed" << std::endl;
            exit(1);
        }

        mSpaceShipTexture = loadTexture("/home/levirs565/Unduhan/SpaceShooterRedux/PNG/playerShip3_blue.png");
        mLaser.texture = loadTexture("/home/levirs565/Unduhan/SpaceShooterRedux/PNG/Lasers/laserBlue01.png");
    }

    void processKeyDown(const SDL_KeyboardEvent& key) {
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

    void processKeyUp(const SDL_KeyboardEvent& key) {
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

            if (mIsFire && mLaser.angle == 0) {
                mLaser.angle = mRotation;
                mLaser.directionVector.x = 0;
                mLaser.directionVector.y = -1;
                mLaser.directionVector.rotate(mRotation * M_PI / 180.0);
                mLaser.position = mPlayerPosition;
            }

            if (mLaser.angle != 0) {
                mLaser.position.add(mLaser.directionVector, 10);
                blit(mLaser.texture, int(mLaser.position.x), int(mLaser.position.y), mLaser.angle);
            }

            presentScene();
            SDL_Delay(16);
        }
    }

    SDL_Texture* loadTexture(char* filename) {
        SDL_LogMessage(SDL_LOG_CATEGORY_APPLICATION, SDL_LOG_PRIORITY_INFO, "Loading texture %s", filename);
        return IMG_LoadTexture(mRenderer, filename);
    }

    void blit(SDL_Texture* texture, int x, int y, double angle) {
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
    Vector2 mPlayerPosition{100, 100};
    Vector2 mDirectionVector{0, 0};
    Laser mLaser;
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