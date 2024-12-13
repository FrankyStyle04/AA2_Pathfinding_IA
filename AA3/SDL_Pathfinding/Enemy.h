class Enemy {
private:
    Vector2D currentPosition; // Posici�n actual del enemigo
    Vector2D startPosition;   // Punto inicial
    Vector2D endPosition;     // Punto final
    bool movingForward;       // Direcci�n de movimiento
    float timeAccumulator;    // Acumulador de tiempo
    float moveInterval;       // Intervalo entre movimientos

public:
    Enemy(Vector2D start, Vector2D end, float interval)
        : startPosition(start), endPosition(end), currentPosition(start),
        movingForward(true), timeAccumulator(0), moveInterval(interval) {}

    // Actualiza la posici�n del enemigo
    void update(float deltaTime) {
        timeAccumulator += deltaTime;
        if (timeAccumulator >= moveInterval) {
            timeAccumulator = 0; // Reinicia el acumulador de tiempo
            if (movingForward) {
                if (currentPosition == endPosition) {
                    movingForward = false; // Cambiar direcci�n
                }
                else {
                    moveTowards(endPosition);
                }
            }
            else {
                if (currentPosition == startPosition) {
                    movingForward = true; // Cambiar direcci�n
                }
                else {
                    moveTowards(startPosition);
                }
            }
        }
    }

    // Mueve al enemigo hacia el destino
    void moveTowards(Vector2D target) {
        if (currentPosition.x < target.x) currentPosition.x++;
        else if (currentPosition.x > target.x) currentPosition.x--;

        if (currentPosition.y < target.y) currentPosition.y++;
        else if (currentPosition.y > target.y) currentPosition.y--;
    }

    // Devuelve la posici�n actual
    Vector2D getPosition() const {
        return currentPosition;
    }
};
