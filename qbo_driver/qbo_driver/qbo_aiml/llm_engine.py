import torch
from transformers import AutoTokenizer, AutoModelForCausalLM


class LLMEngine:

    def __init__(self, model_name: str, logger, enable: bool = True):
        self.logger = logger
        self.enable = enable
        self.model_name = model_name

        self.model = None
        self.tokenizer = None

        if not self.enable:
            self.logger.info("🧠 LLM désactivé.")
            return

        self.logger.info("🔄 Chargement modèle de génération...")

        self.tokenizer = AutoTokenizer.from_pretrained(model_name)

        if self.tokenizer.pad_token is None:
            self.tokenizer.pad_token = self.tokenizer.eos_token

        self.model = AutoModelForCausalLM.from_pretrained(
            model_name,
            torch_dtype=torch.float16,
            device_map="auto"
        ).eval()

        self.logger.info("✅ LLM prêt.")

    # =====================================================
    # STYLE REWRITE (conversation / explain uniquement)
    # =====================================================

    def rewrite(self, base_answer: str) -> str:

        if not self.enable or not base_answer:
            return base_answer

        # 🔒 Sécurité : éviter de toucher aux phrases techniques chiffrées
        if any(char.isdigit() for char in base_answer):
            return base_answer

        messages = [
            {
                "role": "system",
                "content": (
                    "Tu es Qbo, un petit robot mobile curieux et amical.\n"
                    "Tu parles à la première personne.\n"
                    "Tu dois STRICTEMENT reformuler la phrase donnée.\n"
                    "Tu ne dois rien ajouter.\n"
                    "Tu ne dois rien supprimer.\n"
                    "Tu ne dois pas changer le sens.\n"
                    "Tu ne dois pas poser de question.\n"
                    "Tu dois produire UNE SEULE phrase.\n"
                )
            },
            {
                "role": "user",
                "content": base_answer
            }
        ]

        # gen_input = self.tokenizer.apply_chat_template(
        #     messages,
        #     return_tensors="pt"
        # )

        # gen_input = gen_input.to(self.model.device)

        # input_length = gen_input.shape[1]

        text = self.tokenizer.apply_chat_template(
            messages,
            tokenize=False,
            add_generation_prompt=True
        )

        gen_input = self.tokenizer(
            text,
            return_tensors="pt"
        )

        gen_input = {k: v.to(self.model.device) for k, v in gen_input.items()}

        input_length = gen_input["input_ids"].shape[1]

        with torch.no_grad():
            output = self.model.generate(
                **gen_input,
                max_new_tokens=40,  # Limiter la génération à 40 tokens pour éviter les réponses trop longues
                do_sample=True,     # Activer le sampling pour plus de diversité
                temperature=0.5,    # Ajuster la créativité
                top_k=20,           # Garder les 20 tokens les plus probables
                top_p=0.9,          # Garder les tokens cumulativement les plus probables jusqu'à 90%
                repetition_penalty=1.1, # Pénaliser légèrement les répétitions
                pad_token_id=self.tokenizer.eos_token_id, # Utiliser le token EOS comme token de padding
                eos_token_id=self.tokenizer.eos_token_id, # Arrêter la génération à la fin de la séquence
                early_stopping=False, # Arrêter la génération dès que le token EOS est généré

            )

        new_tokens = output[0][input_length:]
        final = self.tokenizer.decode(
            new_tokens,
            skip_special_tokens=True
        ).strip()

        return final or base_answer