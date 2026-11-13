package frc.robot.control;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.security.SecureRandom;
import java.util.HexFormat;
import java.util.Map;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.Executors;
import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpServer;
import edu.wpi.first.wpilibj.DriverStation;

/** Servidor HTTP no porto de equipe. A thread HTTP nunca acessa atuadores. */
public final class TouchServer {
  private static final int PORT = 5800;
  private static final Map<String, String> FILES = Map.of(
      "/", "index.html", "/index.html", "index.html", "/app.js", "app.js",
      "/style.css", "style.css", "/field.png", "field.png",
      "/elastic-layout.json", "../elastic-layout.json");
  private static final Map<String, String> MIME = Map.of(
      "html", "text/html; charset=utf-8", "js", "text/javascript; charset=utf-8",
      "css", "text/css; charset=utf-8", "png", "image/png", "json", "application/json");

  private final ArrayBlockingQueue<Request> requests = new ArrayBlockingQueue<>(16);
  private final String token = HexFormat.of().formatHex(new SecureRandom().generateSeed(16));
  private volatile String status = "{}";
  private long lastAcceptedNanos;

  public TouchServer(Path assets) {
    try {
      HttpServer server = HttpServer.create(new InetSocketAddress(PORT), 8);
      server.setExecutor(Executors.newSingleThreadExecutor(runnable -> {
        Thread thread = new Thread(runnable, "StormTouch");
        thread.setDaemon(true);
        return thread;
      }));
      server.createContext("/", exchange -> handle(exchange, assets));
      server.start();
    } catch (IOException error) {
      DriverStation.reportWarning("Tela STORM indisponível: " + error.getMessage(), false);
    }
  }

  public String token() { return token; }
  public void publish(String snapshot) { status = snapshot; }
  public Request poll() { return requests.poll(); }
  public void clear() { requests.clear(); }

  private void handle(HttpExchange exchange, Path assets) throws IOException {
    try {
      String path = exchange.getRequestURI().getPath();
      String method = exchange.getRequestMethod();
      if (method.equals("GET") && path.equals("/api/status")) {
        respond(exchange, 200, "application/json; charset=utf-8",
            status.getBytes(StandardCharsets.UTF_8));
        return;
      }
      if (method.equals("POST") && path.equals("/api/action")) {
        if (!token.equals(exchange.getRequestHeaders().getFirst("X-Storm-Session"))) {
          exchange.sendResponseHeaders(403, -1);
          return;
        }
        var headers = exchange.getRequestHeaders();
        String length = headers.getFirst("Content-Length");
        int bytes;
        try {
          bytes = Integer.parseInt(length);
        } catch (NumberFormatException error) {
          bytes = -1;
        }
        if (bytes < 1 || bytes > 100) {
          exchange.sendResponseHeaders(400, -1);
          return;
        }
        long now = System.nanoTime();
        if (now - lastAcceptedNanos < 45_000_000L) {
          exchange.sendResponseHeaders(429, -1);
          return;
        }
        byte[] payload = new byte[bytes];
        new java.io.DataInputStream(exchange.getRequestBody()).readFully(payload);
        String body = new String(payload, StandardCharsets.UTF_8);
        if (!body.matches("[a-zA-Z0-9_.: -]{1,100}")) {
          exchange.sendResponseHeaders(400, -1);
          return;
        }
        lastAcceptedNanos = now;
        boolean accepted = requests.offer(new Request(now, body));
        respond(exchange, accepted ? 202 : 503, "text/plain; charset=utf-8",
            (accepted ? "Pedido recebido" : "Fila ocupada").getBytes(StandardCharsets.UTF_8));
        return;
      }
      String asset = FILES.get(path);
      if (!method.equals("GET") || asset == null || !Files.isRegularFile(assets.resolve(asset))) {
        exchange.sendResponseHeaders(404, -1);
        return;
      }
      String extension = asset.substring(asset.lastIndexOf('.') + 1);
      respond(exchange, 200, MIME.get(extension), Files.readAllBytes(assets.resolve(asset)));
    } catch (RuntimeException error) {
      DriverStation.reportWarning("Falha no servidor da tela: " + error, false);
      exchange.sendResponseHeaders(500, -1);
    } finally {
      exchange.close();
    }
  }

  private static void respond(HttpExchange exchange, int statusCode, String mime, byte[] body)
      throws IOException {
    exchange.getResponseHeaders().set("Content-Type", mime);
    exchange.getResponseHeaders().set("Cache-Control", "no-store");
    exchange.getResponseHeaders().set("X-Content-Type-Options", "nosniff");
    exchange.getResponseHeaders().set("Content-Security-Policy",
        "default-src 'self'; script-src 'self'; style-src 'self'; img-src 'self'; frame-ancestors 'none'");
    exchange.sendResponseHeaders(statusCode, body.length);
    exchange.getResponseBody().write(body);
  }

  public record Request(long receivedNanos, String action) {
    public boolean fresh() { return System.nanoTime() - receivedNanos < 500_000_000L; }
  }
}

