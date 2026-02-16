const express = require('express');
const cors = require('cors');
const dotenv = require('dotenv');
const { OpenAI } = require("openai");
const { FaissStore } = require("@langchain/community/vectorstores/faiss");
const { OpenAIEmbeddings } = require("@langchain/openai");
const path = require('path');
const fs = require('fs');
const bcrypt = require('bcryptjs');
const jwt = require('jsonwebtoken');

// Load environment variables
try {
    dotenv.config({ path: path.join(__dirname, '../.env.local') });
    if (!process.env.OPENAI_API_KEY) {
        dotenv.config({ path: path.join(__dirname, '../.env') });
    }
} catch (e) {
    console.log("Dotenv config loading warning: " + e.message);
}

const app = express();
app.use(cors());
app.use(express.json());

const PORT = process.env.PORT || 3001;
const JWT_SECRET = process.env.JWT_SECRET || 'hackathon-secret-key-2026';
const openai = new OpenAI({ apiKey: process.env.OPENAI_API_KEY });
const USERS_FILE = path.join(__dirname, 'users.json');

// Initialize users file if not exists
if (!fs.existsSync(USERS_FILE)) {
    fs.writeFileSync(USERS_FILE, JSON.stringify([]));
}

let vectorStore = null;

async function initVectorStore() {
    try {
        const vectorStorePath = path.join(__dirname, 'vector-store');
        const indexPath = path.join(vectorStorePath, 'faiss.index');
        console.log("Checking vector store at: " + vectorStorePath);

        if (!fs.existsSync(vectorStorePath) || !fs.existsSync(indexPath)) {
            console.log("Vector store files not found. Chat will run in general knowledge mode.");
            return;
        }

        const embeddings = new OpenAIEmbeddings({
            openAIApiKey: process.env.OPENAI_API_KEY,
        });

        vectorStore = await FaissStore.load(vectorStorePath, embeddings);
        console.log("Vector store loaded successfully.");
    } catch (error) {
        console.log("Failed to load vector store: " + error.message);
        vectorStore = null;
    }
}

// --- Auth Endpoints ---

app.post('/auth/signup', async (req, res) => {
    try {
        const { username, email, password } = req.body;
        const users = JSON.parse(fs.readFileSync(USERS_FILE));

        if (users.find(u => u.email === email)) {
            return res.status(400).json({ message: 'User already exists' });
        }

        const hashedPassword = await bcrypt.hash(password, 10);
        const newUser = { id: Date.now(), username, email, password: hashedPassword };
        users.push(newUser);
        fs.writeFileSync(USERS_FILE, JSON.stringify(users));

        res.status(201).json({ message: 'User created successfully' });
    } catch (error) {
        res.status(500).json({ message: 'Signup error', error: error.message });
    }
});

app.post('/auth/signin', async (req, res) => {
    try {
        const { email, password } = req.body;
        const users = JSON.parse(fs.readFileSync(USERS_FILE));
        const user = users.find(u => u.email === email);

        if (!user || !(await bcrypt.compare(password, user.password))) {
            return res.status(401).json({ message: 'Invalid credentials' });
        }

        const token = jwt.sign({ id: user.id, username: user.username }, JWT_SECRET, { expiresIn: '24h' });
        res.json({ token, username: user.username });
    } catch (error) {
        res.status(500).json({ message: 'Signin error', error: error.message });
    }
});

app.post('/auth/forgot', async (req, res) => {
    try {
        const { email } = req.body;
        const users = JSON.parse(fs.readFileSync(USERS_FILE));
        const user = users.find(u => u.email === email);

        if (!user) {
            // Security best practice: don't reveal if user exists, but for hackathon we can be explicit
            return res.status(404).json({ message: 'Email not found' });
        }

        console.log(`[AUTH] Password reset requested for: ${email}`);
        res.json({ message: 'Password reset link sent to your email (Demo: Check server logs)' });
    } catch (error) {
        res.status(500).json({ message: 'Forgot password error', error: error.message });
    }
});

// --- Chat Endpoint ---

app.post('/chat', async (req, res) => {
    try {
        const { query, username } = req.body;
        console.log("Received query from " + (username || "Guest") + ": " + query);

        if (!query) return res.status(400).json({ message: 'Query is required' });

        let context = "";
        if (vectorStore) {
            console.log("Searching for context...");
            const results = await vectorStore.similaritySearch(query, 3);
            context = results.map(r => r.pageContent).join("\n\n---\n\n");
            console.log("Found " + results.length + " chunks.");
        }

        const personality = username ? `The user's name is ${username}. Greet them personally.` : "The user is a Guest.";
        const systemPrompt = `You are an AI assistant for the Physical AI & Humanoid Robotics course. ${personality} 

Language Instructions:
- Detect the language of the user's query.
- If the user asks in Urdu (اردو), respond in clear, professional Urdu.
- If the user asks in English, respond in English.
- Use context if available to provide technical details: ${context}
- Even in Urdu, keep technical terms like "ROS 2", "Gazebo", "Actuator" in English script for clarity.
- Keep answers concise and helpful. Use Markdown formatting.`;

        console.log("Requesting OpenAI...");
        const response = await openai.chat.completions.create({
            model: "gpt-3.5-turbo",
            messages: [
                { role: "system", content: systemPrompt },
                { role: "user", content: query }
            ],
            temperature: 0.7,
        });

        console.log("OpenAI response received.");
        res.json({ answer: response.choices[0].message.content });
    } catch (error) {
        console.log("Chat error: " + error.message);
        res.status(500).json({ message: 'Error', error: error.message });
    }
});

app.get('/health', (req, res) => {
    res.json({ status: 'ok', vectorStoreLoaded: !!vectorStore });
});

app.listen(PORT, "0.0.0.0", () => {
    console.log("Backend running at http://localhost:" + PORT);
    initVectorStore();
});
