package org.usfirst.frc4904.robot.vision;

import java.io.IOException;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.util.ArrayList;
import java.util.List;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class GoogleTagManager {
    private final HttpClient client;

    public record Tag(int id, Rotation2d rot, Translation3d pos, int camera) {}

    public GoogleTagManager() {
        client = HttpClient.newHttpClient();
    }

    public List<Tag> getTags() {
        HttpRequest request = HttpRequest.newBuilder()
            .uri(URI.create("dauntless.local:8080/api/tags"))
            .GET()
            .build();

        List<Tag> tags = new ArrayList<>();

        String json;

        try {
            HttpResponse<String> response = client.send(request, HttpResponse.BodyHandlers.ofString());
            json = response.body();
        } catch (Exception e) {
            System.out.println("google tag manager fetching error!!!");
            return tags;
        }

        try {
            ObjectMapper mapper = new ObjectMapper();
            JsonNode root = mapper.readTree(json);

            for (JsonNode el : root) {
                double[] pos = mapper.treeToValue(el.path("pos"), double[].class);

                Tag tag = new Tag(
                    el.path("id").asInt(),
                    Rotation2d.fromRotations(el.path("rot").asDouble()),
                    new Translation3d(pos[0], pos[1], pos[2]),
                    0
                );

                tags.add(tag);
            }
        } catch (Exception e) {
            System.out.println("google tag manager parsing error!!!");
        }

        return tags;
    }
}
