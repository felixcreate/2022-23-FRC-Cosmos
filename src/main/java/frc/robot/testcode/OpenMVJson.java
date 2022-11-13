package frc.robot.testcode;

import java.util.HashSet;
import java.util.Set;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonMappingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

public class OpenMVJson {
    static ObjectMapper mapper = new ObjectMapper();

    public static class JsonParseException extends Exception {
        public JsonParseException(String e) {
            super(e);
        }
    }

    static class AprilTag {
        int id;
        double x;
        double y;
        double z;
        double rx;
        double ry;
        double rz;
        double confidence;

        public AprilTag(int id, double x, double y, double z, double rx, double ry, double rz, double confidence) {
            this.id = id;
            this.x = x;
            this.y = y;
            this.z = z;
            this.rx = rx;
            this.ry = ry;
            this.rz = rz;
            this.confidence = confidence;
        }
    }
    public static void main(String[] args) throws JsonMappingException, JsonProcessingException, JsonParseException {
        OpenMVJson test = new OpenMVJson("[{\"id\": 3, \"z\": -8.78709, \"rz\": 3.08318, \"confidence\": 0.160103, \"y\": 0.0554678, \"x\": -1.85676, \"ry\": 0.607891, \"rx\": 2.95788}]");
        for(AprilTag tag: test.tags) {
            System.out.println(tag.id);
        }
    }
    
    Set<AprilTag> tags = new HashSet<>();

    public OpenMVJson(String jsonContent) throws JsonMappingException, JsonProcessingException, JsonParseException {
        JsonNode root = mapper.readTree(jsonContent);
        if(!root.isArray()) {
            throw new JsonParseException("Not an json array");
        }
        else {
            for (JsonNode taginfo : root) {
                AprilTag tag = new AprilTag(taginfo.get("id").asInt(), taginfo.get("x").asDouble(), taginfo.get("y").asDouble(), taginfo.get("z").asDouble(), taginfo.get("rx").asDouble(), taginfo.get("ry").asDouble(), taginfo.get("rz").asDouble(), taginfo.get("confidence").asDouble());
                tags.add(tag);
            }
        }
    }
}